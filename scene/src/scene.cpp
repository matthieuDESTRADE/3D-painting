#include "scene.hpp"
#include "interpolation.hpp"


using namespace cgp;




vec3 modulo(vec3 f) {
	while (f.x > 0.5f) f.x -= 1;
	while (f.y > 0.5f) f.y -= 1;
	while (f.x < -0.5f) f.x += 1;
	while (f.y < -0.5f) f.y += 1;

	return f;
}
int geti(int i, int idx, int jdx) {
	if (i < 0)return i + jdx;
	else return i % (jdx);
}
int getj(int j, int idx, int jdx) {
	if (j < 0)return (j + idx * jdx) / (jdx);
	else return j % (idx * jdx) / (jdx);
}


void scene_structure::simulation_step(vec3 gr[], vec3 grv[], int lx, int ly, float dt, float k, float perturbation, float frottements, float scale, bool ciel) {
	float noise = 0;
	float noise1 = 0;
	for (int i = 0; i < lx * ly; i++) {
		float vt = 1;
		noise = noise_perlin({ (modulo(gr[i]).x * scale + ctemps * vt),(modulo(gr[i]).y) * scale }, parameters.octave, parameters.persistency, parameters.frequency_gain);
		noise1 = noise_perlin({ (modulo(gr[i]).x) * scale,(modulo(gr[i]).y * scale + ctemps * vt) }, parameters.octave, parameters.persistency, parameters.frequency_gain);
		noise += 0.5f * noise_perlin({ (modulo(gr[i]).x * scale + ctemps * vt)/ 5,(modulo(gr[i]).y) * scale / 5 }, parameters.octave, parameters.persistency, parameters.frequency_gain);
		noise1 += 0.5f * noise_perlin({ (modulo(gr[i]).x) * scale / 5,(modulo(gr[i]).y * scale + ctemps * vt) / 5 }, parameters.octave, parameters.persistency, parameters.frequency_gain);

		gr[i] += grv[i] * dt;
		grv[i] -= (frottements * grv[i] + perturbation * vec3({ noise - 0.73f, noise1 - 0.75f,0 })) * dt;

		vec3 vec;

		vec = modulo(gr[getj(i, lx, ly) * ly + geti(i + 1, lx, ly)] - gr[i]) ;
		grv[i] += vec * norm(vec) * dt * k;
		vec = modulo(gr[getj(i, lx, ly) * ly + geti(i - 1, lx, ly)] - gr[i]) ;
		grv[i] += vec * norm(vec) * dt * k;
		vec = modulo(gr[getj(i + ly, lx, ly) * ly + geti(i, lx, ly)] - gr[i]) ;
		grv[i] += vec * norm(vec) * dt * k;
		vec = modulo(gr[getj(i - ly, lx, ly) * ly + geti(i, lx, ly)] - gr[i]) ;
		grv[i] += vec * norm(vec) * dt * k;

		if (ciel) {
			grv[i] += vec3({ 0,0.05f,0 }) * dt;
		}


	}
}

void scene_structure::simulation_step3D(vec3 gr[], vec3 grv[], int lx, int ly, float dt, float k, float frottements,vec3 pos) {
	float noise;
	float noise2;
	for (int i = 0; i < lx * ly; i++) {
		noise = noise_perlin({ ctemps*0.2f,0 }, parameters.octave, parameters.persistency, parameters.frequency_gain);
		noise2 = noise_perlin({ 0,ctemps * 0.2f }, parameters.octave, parameters.persistency, parameters.frequency_gain);

		gr[i] += grv[i] * dt;
		grv[i] -= frottements * grv[i];

		vec3 vec;
		float l = .01f*(M-i/N)/(float)M;
		if (i/N != M-1) {
			grv[i] += { 3* (noise2 - .5f) * dt,3*( noise-.5f)*dt + .5f*dt, -dt*0.5f };

			vec = gr[getj(i + ly, lx, ly) * ly + geti(i, lx, ly)] - gr[i];
			grv[i] += vec * (norm(vec) - l) * dt * k;
			if (i / N != 0) {
				vec = gr[getj(i - ly, lx, ly) * ly + geti(i, lx, ly)] - gr[i];
				grv[i] += vec * (norm(vec) - l) * dt * k;
			}
			vec = gr[getj(i, lx, ly) * ly + geti(i + 1, lx, ly)] - gr[i];
			grv[i] += vec * (norm(vec) - l) * dt * k;
			vec = gr[getj(i, lx, ly) * ly + geti(i - 1, lx, ly)] - gr[i];
			grv[i] += vec * (norm(vec) - l) * dt * k;

			vec3 test = gr[i]+ grv[i] * dt - pos;
			test.z = 0;
			if (norm(test) < 0.03f)//éviter les jambes
			{
				grv[i] += normalize(test)* (- norm(test) + 0.03) * 50;
			}
			
		}
	}
}




// This function is called only once at the beginning of the program
// This function can contain any complex operation that can be pre-computed once
void scene_structure::initialize()
{
	std::cout << "Start function scene_structure::initialize()" << std::endl;

	// Set the behavior of the camera and its initial position
	// ********************************************** //
	camera_control2.initialize(inputs, window);
	camera_control2.set_rotation_axis_z(); // camera rotates around z-axis
	//   look_at(camera_position, targeted_point, up_direction)
	camera_control2.look_at(
		{ 3.0f, -2.0f, 1.5f } /* position of the camera in the 3D scene */,
		{ 0,0,0 } /* targeted point in 3D scene */,
		{ 0,0,1 } /* direction of the "up" vector */);


	// Create the global (x,y,z) frame
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());

	opengl_shader_structure shaderSingleColor;
	shaderSingleColor.load(
	project::path + "shaders/shaderSingleColor/shaderSingleColor.vert.glsl",
	project::path + "shaders/shaderSingleColor/shaderSingleColor.frag.glsl");

	opengl_shader_structure shaderSingleColor2;
	shaderSingleColor2.load(
	project::path + "shaders/shaderSingleColor2/shaderSingleColor2.vert.glsl",
	project::path + "shaders/shaderSingleColor2/shaderSingleColor2.frag.glsl");

	opengl_shader_structure shaderSingleColor3;
	shaderSingleColor3.load(
	project::path + "shaders/shaderSingleColor3/shaderSingleColor3.vert.glsl",
	project::path + "shaders/shaderSingleColor3/shaderSingleColor3.frag.glsl");

	opengl_shader_structure eglise_shader;
	eglise_shader.load(
	project::path + "shaders/eglise/eglise.vert.glsl",
	project::path + "shaders/eglise/eglise.frag.glsl");

	// Create the shapes seen in the 3D scene
	// ********************************************** //
	//MESH OF THE INSIVIBLE CUBE
	mesh cube_mesh = mesh_primitive_cube(/*center*/{ 0,0,0 }, /*edge length*/ 1.2f);
	cubee.initialize_data_on_gpu(cube_mesh);
	cubee.material.color = { 1,1,0 };  

	//MESH OF THE SPHERE BLUE BACKGROUND
	mesh sphere_mesh = mesh_primitive_sphere();
	sphere.initialize_data_on_gpu(sphere_mesh);
	sphere.model.scaling = 30.0f;
	sphere.model.translation = { 0,0,0 };
	sphere.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/ciel.png");

	//FRAME
	mesh frame_mesh = mesh_primitive_cube(/*center*/{ 0,0,0 }, /*edge length*/ 1.25f);
	frame.initialize_data_on_gpu(frame_mesh);
	frame.shader = shaderSingleColor;

	mesh frame_mesh2 = mesh_primitive_cube(/*center*/{ 0,0,0 }, /*edge length*/ 1.4f);
	frame2.initialize_data_on_gpu(frame_mesh2);
	frame2.shader = shaderSingleColor2;

	mesh frame_mesh3 = mesh_primitive_cube(/*center*/{ 0,0,0 }, /*edge length*/ 1.46f);
	frame3.initialize_data_on_gpu(frame_mesh3);
	frame3.shader = shaderSingleColor3;


	//TERRAIN
	terrain.initialize_data_on_gpu(terrain_mesh);
	terrain.material.phong.specular = 0.0f;
	terrain.model.translation = { 0,0,-.8f };
	terrain.model.scaling_xyz = { 2,2,1 };
	terrain.material.color = {0.71,0.71,0.32};

	//GUI
	project::gui_scale = 1.5f;

	auto struct_shape = mesh_load_file_obj_advanced(project::path + "assets/eglisee/", "eglisee.obj");
	shapes = mesh_obj_advanced_loader::convert_to_mesh_drawable(struct_shape);
	for (int k = 0; k < shapes.size(); ++k){
		shapes[k].model.scaling = 0.5f;
		shapes[k].model.rotation = rotation_transform::from_axis_angle({ 0,0,1 }, 3.14 / 4);
		shapes[k].model.translation = { 0.1f,0.1f,-0.2f };
		shapes[k].material.phong.specular = 0.0f;
		shapes[k].shader = eglise_shader;

	}

	//MAISONS
	auto maison1_shape = mesh_load_file_obj_advanced(project::path + "assets/maison1/", "maison1.obj");
	shapes_maison1 = mesh_obj_advanced_loader::convert_to_mesh_drawable(maison1_shape);
	for (int k = 0; k < shapes_maison1.size(); ++k) {
		shapes_maison1[k].model.scaling = 0.08f;
		// shapes_maison1[k].model.rotation = rotation_transform::from_axis_angle({ 0,0,1 }, 3.14 / 4);
		shapes_maison1[k].model.translation = { 2.0f,3.0f,-0.30f };
		shapes_maison1[k].material.phong.specular = 0.0f;
		//shapes_maison1[k].shader = eglise_shader;

	}
	auto maison2_shape = mesh_load_file_obj_advanced(project::path + "assets/maison2/", "maison2.obj");
	shapes_maison2 = mesh_obj_advanced_loader::convert_to_mesh_drawable(maison2_shape);
	for (int k = 0; k < shapes_maison2.size(); ++k) {
		shapes_maison2[k].model.scaling = 0.10f;
		// shapes_maison1[k].model.rotation = rotation_transform::from_axis_angle({ 0,0,1 }, 3.14 / 6);
		shapes_maison2[k].model.translation = { 0.8f,-0.6f,-0.15f };
		shapes_maison2[k].material.phong.specular = 0.0f;
		//shapes_maison2[k].shader = eglise_shader;

	}
	auto maison3_shape = mesh_load_file_obj_advanced(project::path + "assets/maison3/", "maison3.obj");
	shapes_maison3 = mesh_obj_advanced_loader::convert_to_mesh_drawable(maison3_shape);
	for (int k = 0; k < shapes_maison3.size(); ++k) {
		shapes_maison3[k].model.scaling = 0.10f;
		// shapes_maison1[k].model.rotation = rotation_transform::from_axis_angle({ 0,0,1 }, 3.14 / 4);
		shapes_maison3[k].model.translation = { -1.8f,-1.3f,-0.3f };
		shapes_maison3[k].material.phong.specular = 0.0f;
		//shapes_maison3[k].shader = eglise_shader;

	}

	//FEMME
	mesh corps = mesh_load_file_obj(project::path + "assets/femmecorps.obj");
	corps_mesh.initialize_data_on_gpu(corps);
	corps_mesh.shader.load(project::path + "shaders/mesh/mesh.vert.glsl", project::path + "shaders/mesh/mesh.frag.glsl");
	corps_mesh.material.phong.specular = 0;
	corps_mesh.material.color = { 1,1,1 };
	corps_mesh.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/corps.png");

	mesh tete = mesh_load_file_obj(project::path + "assets/tete.obj");
	tete_mesh.initialize_data_on_gpu(tete);
	tete_mesh.shader.load(project::path + "shaders/mesh/mesh.vert.glsl", project::path + "shaders/mesh/mesh.frag.glsl");
	tete_mesh.material.phong.specular = 0;
	tete_mesh.material.color = { 1,1,1 };
	tete_mesh.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/corps.png");

	mesh bras = mesh_load_file_obj(project::path + "assets/bras.obj");
	bras_mesh.initialize_data_on_gpu(bras);
	bras_mesh.shader.load(project::path + "shaders/mesh/mesh.vert.glsl", project::path + "shaders/mesh/mesh.frag.glsl");
	bras_mesh.material.phong.specular = 0;
	bras_mesh.material.color = { 1,1,1 };
	bras_mesh.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/corps.png");

	femme.add(corps_mesh, "corps");
	femme.add(bras_mesh, "bras","corps");
	femme.add(bras_mesh, "avantbras", "bras");
	femme.add(bras_mesh, "bras2", "corps");
	femme.add(bras_mesh, "avantbras2", "bras2");
	femme.add(tete_mesh, "tete", "corps");
	femme["tete"].transform_local.translation = {0,4.1,0};
	femme["bras"].transform_local.translation = { -1.26,3.4,0 };
	femme["bras2"].transform_local.translation = { 1.26,3.4,0 };
	femme["avantbras"].transform_local.translation = { 0,-2.4,0 };
	femme["avantbras2"].transform_local.translation = { 0,-2.4,0 };
	femme["corps"].transform_local.rotation = rotation_transform::from_axis_angle({ 1,0,0 }, 3.1415 / 2);
	femme["corps"].transform_local.scaling = .015f * 0.7f;
	femme["corps"].transform_local.translation = {0.5f,0,0};

	//ROBE
	robe.position.resize(N * M);
	robe.uv.resize(N * M);
	for (int ku = 0; ku < M; ++ku)
	{
		for (int kv = 0; kv < N; ++kv)
		{
			robegrille[kv + N*ku] = { 0.02f*cos(2 * 3.1415 * kv / (float)N),0.02f*sin(2 * 3.1415 * kv / (float)N),-0.06f + 0.07f*ku/(float)M};
			robe.position[kv + N * ku] = robegrille[kv + N * ku];
			robe.uv[kv + N * ku] = { kv /(float) N,ku /(float) M };
			robevitesse[kv + N * ku] = {0,0,0};
		}
	}
	// Generate triangle organization
	for (int ku = 0; ku < M - 1; ++ku)
	{
		for (int kv = 0; kv < N - 1; ++kv)
		{
			unsigned int idx = kv + N * ku; // current vertex offset

			uint3 triangle_1 = {  idx + 1 + N, idx,idx + 1 };
			uint3 triangle_2 = {  idx + N, idx,idx + 1 + N };

			robe.connectivity.push_back(triangle_1);
			robe.connectivity.push_back(triangle_2);
		}
		robe.connectivity.push_back({N-1 + N*ku, N*ku, N-1 + N*(ku+1)});
		robe.connectivity.push_back({ N - 1 + N * (ku+1), N * ku, N * (ku+1) });
	}

	// need to call this function to fill the other buffer with default values (normal, color, etc)
	robe.fill_empty_field();
	robe_mesh.initialize_data_on_gpu(robe);
	robe_mesh.shader.load(project::path + "shaders/mesh/mesh.vert.glsl", project::path + "shaders/mesh/mesh.frag.glsl");
	robe_mesh.material.phong.specular = 0;

	//ARBRES 
	tree_position = { vec2(-1.0f,-1.0f), vec2(1.0f,0) };

	opengl_shader_structure tree_shader;
	tree_shader.load(
		project::path + "shaders/tree_shader/tree_shader.vert.glsl",
		project::path + "shaders/tree_shader/tree_shader.frag.glsl");

	for (int k = 0; k < tree_position.size(); k++) {
		mesh tree_mesh = mesh_primitive_sphere(0.1f);
		tree_meshes[k].initialize_data_on_gpu(tree_mesh);
		float x = tree_position[k].x;
		float y = tree_position[k].y;
		tree_meshes[k].model.translation = { x,y,-0.25};
		tree_meshes[k].material.phong.specular = 0.0f;
		tree_meshes[k].shader = tree_shader;
		tree_meshes[k].texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/TreeTexture.png");
	}


	// MESH BILLBOARDS -----------------------------------------------------------------------------------------------------------------------------------
	mesh plane_mesh = mesh_primitive_quadrangle({ -1,0,-1 }, { 1,0,-1 }, { 1,0,1 }, { -1,0,1 });
	plane.initialize_data_on_gpu(plane_mesh);
	plane.model.scaling = 3.0f;
	plane.material.phong.specular = 0;

	//CIEL
	plane.model.scaling = 2;
	plane.shader.load(project::path + "shaders/instancingCiel/instancing.vert.glsl", project::path + "shaders/instancingCiel/instancing.frag.glsl");
	planeciel = plane;
	planeciel.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/cieltexture.png");

	//SOL
	plane.model.scaling = 0.05f;
	planesol = plane;
	planesol.shader.load(project::path + "shaders/instancing/instancing.vert.glsl", project::path + "shaders/instancing/instancing.frag.glsl");
	planesol.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/soltexture.png");

	//CHEMIN
	planechemin = plane;
	planechemin.shader.load(project::path + "shaders/instancingChemin/instancing.vert.glsl", project::path + "shaders/instancingChemin/instancing.frag.glsl");
	planechemin.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/chemintexture.png");

	//===================================================================================================================================================================================================================

	// TIMER
	timer.t_min = 1;
	timer.t_max = 10000000;
	timer.t = timer.t_min;

	//LISTES D'INSTANCING BILLBOARDS ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	//CIEL
	for (int i = 0; i < nbtaches * 2; i++) {
		for (int j = 0; j < nbtaches; j++) {
			grille[i + nbtaches * 2 * j] = { (1 / (float)(nbtaches * 2)) * i,(1 / (float)nbtaches) * j,0 };
			textureid[i + nbtaches * 2 * j] = (int)std::rand() % 4;
			grillevitesse[i + nbtaches * 2 * j] = { 0,0,0 };
		}
	}
	posciel.resize(nbtaches * nbtaches * 2);
	rotcielx.resize(nbtaches * nbtaches * 2);
	rotciely.resize(nbtaches * nbtaches * 2);
	rotcielz.resize(nbtaches * nbtaches * 2);
	colciel.resize(nbtaches * nbtaches * 2);
	textciel.resize(nbtaches * nbtaches * 2);

	//SOL
	for (int i = 0; i < nbtachessol; i++) {
		for (int j = 0; j < nbtachessol; j++) {
			grillesol[i + nbtachessol * j] = { (1 / (float)(nbtachessol)) * i,(1 / (float)nbtachessol) * j,0 };
			textureidsol[i + nbtachessol * j] = (int)std::rand() % 6 %4;
			grillevitessesol[i + nbtachessol * j] = { 0,0,0 };
		}
	}
	instance_positions.resize(nbtachessol * nbtachessol);
	rotsolx.resize(nbtachessol * nbtachessol);
	rotsoly.resize(nbtachessol * nbtachessol);
	rotsolz.resize(nbtachessol * nbtachessol);
	colsol.resize(nbtachessol * nbtachessol);
	textsol.resize(nbtachessol * nbtachessol);

	//CHEMIN
	chemin.resize(splinePoints.size());
	textidchemin.resize(nbptschemin);

	for (int i = 0; i < splinePoints.size(); i++) {
		//conversion spline -> coordonnées grille du sol dans chemin
		int x = (splinePoints.at(i).x + 1)* nbtachessol / 2;
		int y = (splinePoints.at(i).y + 1) * nbtachessol / 2;
		chemin[i] = x + nbtachessol * y;
		for (int j=-1;j<=1;j++){ 
			for (int k =-1; k <= 1; k++) {
				textureidsol[x + j + nbtachessol * (y+k)] += 4;
			}
		}
	}
	for (int i = 0; i < nbptschemin; i++) {
		textidchemin[i] = (int)std::rand() % 2;
	}
	instanceChemin.resize(nbptschemin);
	rotcheminx.resize(nbptschemin);
	rotcheminy.resize(nbptschemin);
	rotcheminz.resize(nbptschemin);
	textchemin.resize(nbptschemin);

	SplineChemin.resize(chemin.size() + 4);
	key_times.resize(chemin.size() + 3);
	for (int i = 0; i < chemin.size() + 3; i++) {
		key_times[i] = i;
	}
	//=======================================================================================================================================================================
}


// This function is called permanently at every new frame
// Note that you should avoid having costly computation and large allocation defined there. This function is mostly used to call the draw() functions on pre-existing data.
void scene_structure::display_frame()
{
	if (gui.cadre) {
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_STENCIL_TEST);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f); //set background color to black
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);



		glStencilFunc(GL_ALWAYS, 1, 0xFF);
		glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
		glDepthMask(GL_FALSE);

		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

		draw(cubee, environment);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

		glDepthMask(GL_TRUE);
		glStencilFunc(GL_EQUAL, 1, 0xFF);
		glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
	}


	//ANIMATION FEMME -------------------------------------------------------------------------------------------------------------------------------------



	femme["bras"].transform_local.rotation = rotation_transform::from_axis_angle({ 0,0,1 }, -(sin(ctemps*2) + 2)* 1 / 4);
	femme["avantbras"].transform_local.rotation = rotation_transform::from_axis_angle({ 0,0,1 }, 2 * (sin(ctemps *2) + 2) * 1 / 4);

	femme["bras2"].transform_local.rotation = rotation_transform::from_axis_angle({ 0,0,1 }, (sin(ctemps *2) +2) * 1 / 4);
	femme["avantbras2"].transform_local.rotation = rotation_transform::from_axis_angle({ 0,0,1 }, -2 * (sin(ctemps *2) + 2) * 1 / 4);
	femme.update_local_to_global_coordinates();
	draw(femme, environment);
	robe_mesh.clear();
	robe_mesh.initialize_data_on_gpu(robe);
	//robe_mesh.model.translation = femme["corps"].transform_local.translation;
	robe_mesh.model.scaling = 0.7f;
	robe_mesh.material.phong.specular = 0;
	robe_mesh.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/robe.png");

	draw(robe_mesh, environment);
	//=======================================================================================================================================================

	draw(terrain,environment);
    for (int k = 0; k < shapes.size(); ++k)
		draw(shapes[k], environment);
	for (int k = 0; k < shapes_maison1.size(); ++k)
		draw(shapes_maison1[k], environment);
	for (int k = 0; k < shapes_maison2.size(); ++k)
		draw(shapes_maison2[k], environment);
	for (int k = 0; k < shapes_maison3.size(); ++k)
		draw(shapes_maison3[k], environment);
	for (int k = 0; k < tree_position.size(); ++k)
		draw(tree_meshes[k], environment);
	draw(sphere, environment);
	display_semiTransparent();

	if (gui.vue1)
	{
		camera_control2.look_at(
			{ 2.0f, 2.0f, -0.5f } /* position of the camera in the 3D scene */,
			{ 0,0,0 } /* targeted point in 3D scene */,
			{ 0,0,1 } /* direction of the "up" vector */);
	}


	if (gui.cadre) {
		glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
		draw(frame, environment);

		glStencilFunc(GL_ALWAYS, 2, 0xFF);
		glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
		glDepthMask(GL_FALSE);
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		draw(frame, environment);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDepthMask(GL_TRUE);


		glStencilFunc(GL_NOTEQUAL, 2, 0xFF);
		draw(frame2, environment);

		glStencilFunc(GL_ALWAYS, 3, 0xFF);
		glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
		glDepthMask(GL_FALSE);
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		draw(frame2, environment);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDepthMask(GL_TRUE);


		glStencilFunc(GL_NOTEQUAL, 3, 0xFF);
		draw(frame3, environment);


		// conditional display of the global frame (set via the GUI)
		// if (gui.display_frame)
		// 	draw(global_frame, environment);

		glDisable(GL_DEPTH_TEST);
		glDisable(GL_STENCIL_TEST);
	}


}


void scene_structure::display_semiTransparent()
{
	// Enable use of alpha component as color blending for transparent elements
	//  alpha = current_color.alpha
	//  new color = previous_color * alpha + current_color * (1-alpha)
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Disable depth buffer writing
	//  - Transparent elements cannot use depth buffer
	//  - They are supposed to be display from furest to nearest elements
	glDepthMask(false);

	auto const& camera = camera_control2.camera_model;
	

	// METS A JOUR LE TEMPS (ctemps) S'IL N'Y A PAS EU DE PAUSE TROP LONGUE OU DE FREEZE
	timer.update();
	float delta = timer.t-t;
	delta = std::min(delta, 0.09f);
	ctemps += delta;
	//Envoie le temps au shader
	environment.uniform_generic.uniform_float["time"] = ctemps;
	environment.uniform_generic.uniform_vec3["femme"] = femme["corps"].transform_local.translation;
	t = timer.t;

	//SIMULATION
	simulation_step(grille, grillevitesse, nbtaches, nbtaches * 2, delta*0.1f, gui.cielk, gui.cielts,0.2f,gui.cielf,true);
	simulation_step(grillesol, grillevitessesol, nbtachessol, nbtachessol, delta * 0.1f, gui.solk, gui.solts, 0.3f,gui.solf,false);

	//AFFICHAGE CIEL --------------------------------------------------------------------------------------------------------------------------------------------------
	float theta;
	float pi = atan(1) * 4;
	float phi;
	float rayon = 9.0f;

	for (int i = 0; i < nbtaches * nbtaches * 2; i++) {
		theta = (grille[i].x - (int)grille[i].x) * pi * 2;
		phi = (grille[i].y - (int)grille[i].y) * pi;
		rotation_transform R = rotation_transform::from_frame_transform({ cos(i*5),0,sin(i*5) }, { 0,1,0 }, camera.right(), camera.front());
		posciel[i] = { cos(theta) * sin(phi) * rayon, rayon * sin(theta) * sin(phi), rayon * cos(phi) };
		rotcielx[i] = R.matrix().at(0);
		rotciely[i] = R.matrix().at(1);
		rotcielz[i] = R.matrix().at(2);
		textciel[i] = { textureid[i],0 };
		colciel[i] = vec3(1,1,1) * (i+ nbtaches * nbtaches) /(nbtaches * nbtaches * 3);


	}

	planeciel.initialize_supplementary_data_on_gpu(posciel, /*location*/ 4, /*divisor: 1=per instance, 0=per vertex*/ 1);
	planeciel.initialize_supplementary_data_on_gpu(rotcielx, /*location*/ 5, /*divisor: 1=per instance, 0=per vertex*/ 1);
	planeciel.initialize_supplementary_data_on_gpu(rotciely, /*location*/ 6, /*divisor: 1=per instance, 0=per vertex*/ 1);
	planeciel.initialize_supplementary_data_on_gpu(rotcielz, /*location*/ 7, /*divisor: 1=per instance, 0=per vertex*/ 1);
	planeciel.initialize_supplementary_data_on_gpu(colciel, /*location*/ 8, /*divisor: 1=per instance, 0=per vertex*/ 1);
	planeciel.initialize_supplementary_data_on_gpu(textciel, /*location*/ 9, /*divisor: 1=per instance, 0=per vertex*/ 1);
	draw(planeciel, environment, nbtaches * nbtaches * 2);
	//===============================================================================================================================================================================



	//AFFICHAGE SOL --------------------------------------------------------------------------------------------------------------------------------------------------
	float tilesize = 40;

	for (int i = 0; i < nbtachessol * nbtachessol; i++) {
		instance_positions[i] = (grillesol[i]-vec3(.5f,.5f,0) )* tilesize;
		float noise = std::abs(noise_perlin({ grillesol[i].x,grillesol[i].y }, parameters.octave, parameters.persistency, parameters.frequency_gain));
		rotation_transform R = rotation_transform::from_frame_transform({ cos(i*5+t*noise*.5f),0,sin(i*5+t*noise*.5f) }, { 0,1,0 }, camera.right(), camera.front());
		R = R*rotation_transform::from_axis_angle({ 0,0,1 }, 0.1f*i/( nbtachessol * nbtachessol));
		rotsolx[i] = R.matrix().at(0);
		rotsoly[i] = R.matrix().at(1);
		rotsolz[i] = R.matrix().at(2);
		colsol[i] = { 1,1,1 };
	}

	planesol.initialize_supplementary_data_on_gpu(instance_positions, /*location*/ 4, /*divisor: 1=per instance, 0=per vertex*/ 1);
	planesol.initialize_supplementary_data_on_gpu(rotsolx, /*location*/ 5, /*divisor: 1=per instance, 0=per vertex*/ 1);
	planesol.initialize_supplementary_data_on_gpu(rotsoly, /*location*/ 6, /*divisor: 1=per instance, 0=per vertex*/ 1);
	planesol.initialize_supplementary_data_on_gpu(rotsolz, /*location*/ 7, /*divisor: 1=per instance, 0=per vertex*/ 1);
	planesol.initialize_supplementary_data_on_gpu(colsol, /*location*/ 8, /*divisor: 1=per instance, 0=per vertex*/ 1);

	numarray<vec3> additivepositions;
	additivepositions.resize(nbtachessol * nbtachessol);

	// SETUP SPLINE CHEMIN
	glDepthMask(true);

	float vitesse = gui.v;
	for (int nbtiles = -2; nbtiles < 3; nbtiles++) {
		for (int nbtilesy = -2; nbtilesy < 3; nbtilesy++) {
			int distance;
			if (gui.renderdistance) {
				distance = norm(camera.position() - vec3({ nbtiles,nbtilesy,camera.position().z }));
				srand(NULL);
			}
			else distance = 0;
			if (nbtiles == 0 && nbtilesy == 0) distance = 1;
			int cpt = 0;
			// MAJ TILE SOL
			for (int i = 0; i < nbtachessol * nbtachessol; i += 1 + std::rand() % (std::max(1, (distance * distance / 4) - 1)))
			{
				std::tuple<float, bool> res = evaluate_terrain_height((nbtiles * tilesize + instance_positions[i].x) / 40, (nbtilesy * tilesize + instance_positions[i].y) / 40);
				if (std::get<1>(res)) {

					additivepositions[cpt] = instance_positions[i] + vec3({ nbtiles * tilesize, nbtilesy * tilesize, 0 });
					additivepositions[cpt].z = std::get<0>(res) * 40 - 15;
					if (nbtiles != 0 || nbtilesy != 0)
						textsol[cpt] = { textureidsol[i] % 4,0 };
					else
						textsol[cpt] = { textureidsol[i],0 };

					cpt++;
					

				}
			}

			//MAJ CHEMIN
			if (nbtiles == 0 && nbtilesy == 0) {
				for (int i = 0; i < chemin.size()-1; i++) {
					SplineChemin[i+2] = additivepositions[chemin[i]];

				}
				SplineChemin[0] = additivepositions[chemin[chemin.size() -2]];
				SplineChemin[chemin.size() +1] = additivepositions[chemin[0]];
				SplineChemin[1] = additivepositions[chemin[chemin.size() - 1]];
				for (int i = 0; i < nbptschemin; i++) {
					float ti = 2+ ctemps * vitesse + i * chemin.size()/ (float) nbptschemin;
					while (ti >= 1+(chemin.size())) ti -= (chemin.size()-1);
					vec3 v = interpolationdir(ti, SplineChemin, key_times);
					rotation_transform R;
					if (norm(v) > 0.001f) {
						vec3 w;
						w = normalize(cross({ 0,0,1 }, v));
						R = rotation_transform::from_frame_transform({ 1,0,0 }, { 0,1,0 }, normalize(v), normalize(cross(v, w)));
						
					}
					rotcheminx[i] = R.matrix().at(0);
					rotcheminy[i] = R.matrix().at(1);
					rotcheminz[i] = R.matrix().at(2);
					textchemin[i] = { textidchemin[i],0 };
					instanceChemin[i] = interpolation(ti, SplineChemin, key_times);
				}

				//DRAW CHEMIN
				planechemin.initialize_supplementary_data_on_gpu(instanceChemin, /*location*/ 4, /*divisor: 1=per instance, 0=per vertex*/ 1);
				planechemin.initialize_supplementary_data_on_gpu(textchemin, /*location*/ 13, /*divisor: 1=per instance, 0=per vertex*/ 1);
				planechemin.initialize_supplementary_data_on_gpu(rotcheminx, /*location*/ 10, /*divisor: 1=per instance, 0=per vertex*/ 1);
				planechemin.initialize_supplementary_data_on_gpu(rotcheminy, /*location*/ 11, /*divisor: 1=per instance, 0=per vertex*/ 1);
				planechemin.initialize_supplementary_data_on_gpu(rotcheminz, /*location*/ 12, /*divisor: 1=per instance, 0=per vertex*/ 1);
				planechemin.model.translation = { 0,0,0 };
				draw(planechemin, environment, nbptschemin);
				planechemin.model.translation = { 0.05f,0,0};
				draw(planechemin, environment, nbptschemin);
				planechemin.model.translation = { 0,0.05f,0};
				draw(planechemin, environment, nbptschemin);
				planechemin.model.translation = { 0,-0.05f,0 };
				draw(planechemin, environment, nbptschemin);
				planechemin.model.translation = { -0.05f,0,0 };
				draw(planechemin, environment, nbptschemin);

			}
			//DRAW SOL
			planesol.initialize_supplementary_data_on_gpu(textsol, /*location*/ 9, /*divisor: 1=per instance, 0=per vertex*/ 1);
			planesol.initialize_supplementary_data_on_gpu(additivepositions , /*location*/ 4, /*divisor: 1=per instance, 0=per vertex*/ 1);
			draw(planesol, environment, cpt);
		}


		//MOUVEMENT FEMME/ROBE
		float ti = 5+ ctemps* 0.09f;
		while (ti >= 1 + (chemin.size())) ti -= (chemin.size() - 1);

		if (t > 7) {
			vec3 v = interpolationdir(ti, SplineChemin, key_times);
			rotation_transform R;
			if (norm(v) > 0.001f) {
				vec3 w;
				w = normalize(cross({ 0,0,1 }, v));
				R = rotation_transform::from_frame_transform({ 0,0,1 }, { 0,1,0 }, normalize(vec3(v.x, v.y, 0)), { 0,0,1 });
				femme["corps"].transform_local.rotation = R;
			}
			vec3 sav = femme["corps"].transform_local.translation;
			vec3 mvmt = interpolation(ti, SplineChemin, key_times) / 20;
			femme["corps"].transform_local.translation = mvmt;
			float noise1 = 0.02f * (noise_perlin({ ctemps * 0.2f,0 }, parameters.octave, parameters.persistency, parameters.frequency_gain) - .5f);
			float noise2 = 0.02f * (noise_perlin({ 0 ,ctemps * 0.2f }, parameters.octave, parameters.persistency, parameters.frequency_gain) - .5f);
			femme["corps"].transform_local.translation += {noise1,noise2,0.12f * 0.7f - sin(ctemps *2)*0.005f};
			if (norm(femme["corps"].transform_local.translation - sav) > .5f) {
				for (int i = 0; i < N * M; i++) robegrille[i] += (femme["corps"].transform_local.translation) / 0.7f;
			}
			else {
				for (int i = 0; i < N; i++) robegrille[i + N * (M - 1)] += (femme["corps"].transform_local.translation - sav) / 0.7f;
			}
			simulation_step3D(robegrille, robevitesse, M, N, delta * 0.1f, 100000, 0.3f, (femme["corps"].transform_local.translation) / 0.7f);
			for (int i = 0; i < M * N; i++) robe.position[i] = robegrille[i];
		}
	}
	
	// Don't forget to re-activate the depth-buffer write
	glDepthMask(true);
	glDisable(GL_BLEND);
}




void scene_structure::display_gui()
{
	ImGui::SliderFloat(" ciel turbulence strength", &gui.cielts, 0.0f, 2.0f);
	ImGui::SliderFloat("sol turbulence strength", &gui.solts, 0.0f, 2.0f);
	ImGui::SliderFloat("ciel rigidité", &gui.cielk, 0.0f, 800);
	ImGui::SliderFloat("sol rigidité", &gui.solk, 0.0f, 800);
	ImGui::SliderFloat("ciel frequence", &gui.cielf, 0.0f, 10);
	ImGui::SliderFloat("sol frequence", &gui.solf, 0.0f, 10);
	ImGui::SliderFloat("chemin vitesse", &gui.v, 0.5f, 5);
	ImGui::Checkbox("cadre", &gui.cadre);
	ImGui::Checkbox("distance de rendu adaptative", &gui.renderdistance);
	ImGui::Checkbox("vue 1", &gui.vue1);


}

void scene_structure::mouse_move_event()
{
	if (!inputs.keyboard.shift)
		camera_control2.action_mouse_move(environment.camera_view);

}
void scene_structure::mouse_click_event()
{
	camera_control2.action_mouse_click(environment.camera_view);
}
void scene_structure::keyboard_event()
{
	if (true) {
		camera_control2.action_keyboard(environment.camera_view);
	}
	else{/*
	 	//CHANGER LA MAGNITUDE MAIS C'EST HARDCODE DANS CGP...
	 	//Modify field of view
	 	camera_projection.field_of_view = Pi / 2.0f;
	 	camera_projection.depth_max = 40.0f;
	 	camera_projection.depth_min = 0.0001f;
	 	float z = -0.02f;
	 	vec2 camera_xy = vec2(camera_control2.camera_model.position_camera.x, camera_control2.camera_model.position_camera.y);
	 	for (const auto& vec : ground_pos) {
	 		vec2 terrain_xy = vec2(vec[0],vec[1]);
	 		vec2 dist = camera_xy - terrain_xy;
	 		if (sqrt(dist[0]*dist[0] + dist[1]*dist[1])<=0.001) {
	 			z = vec.z-0.3f;  // Found the matching (x, y), return the corresponding z-coordinate
	 			break;
	 		}
     	}
	 	std::cout << z << std::endl;
	 	camera_control2.camera_model.position_camera.z = z;*/
	}
}
void scene_structure::idle_frame()
{
	camera_control2.idle_frame(environment.camera_view);
}

