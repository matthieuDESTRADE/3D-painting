#pragma once


#include "cgp/cgp.hpp"
#include "environment.hpp"
#include <vector>
#include "key_positions_structure.hpp"
#include "terrain.hpp"

// This definitions allow to use the structures: mesh, mesh_drawable, etc. without mentionning explicitly cgp::
using cgp::mesh;
using cgp::mesh_drawable;
using cgp::vec3;
using cgp::numarray;
using cgp::timer_basic;

// Variables associated to the GUI (buttons, etc)
struct gui_parameters {
	bool display_frame = true;
	float cielts = 0.5f;
	float cielk = 30;

	float solts = 0.6f;
	float solk = 500;
	float cielf = 1;
	float solf = 1;

	float v = 1;
	bool cadre =true;

	float param;
	bool vue1;
	bool renderdistance;

};

// The structure of the custom scene
struct scene_structure : cgp::scene_inputs_generic {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	camera_controller_2d_displacement camera_control1;
	camera_controller_orbit_euler camera_control2;
	camera_projection_perspective camera_projection;
	window_structure window;

	mesh_drawable global_frame;          // The standard global frame
	environment_structure environment;   // Standard environment controler
	input_devices inputs;                // Storage for inputs status (mouse, keyboard, window dimension)
	gui_parameters gui;  
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	std::vector<mesh_drawable> shapes;
	std::vector<mesh_drawable> shapes_maison1;
	std::vector<mesh_drawable> shapes_maison2;
	std::vector<mesh_drawable> shapes_maison3;
	mesh_drawable terrain;
	mesh_drawable cubee;
	mesh_drawable frame;
	mesh_drawable frame2;
	mesh_drawable frame3;
	mesh_drawable sphere;
	mesh_drawable eglise;
	mesh_drawable scaled_up_eglise;
	mesh_drawable sphere_outline;
	mesh_drawable tree_meshes[2];
	std::vector < cgp::vec2 > tree_position; 

	keyframe_structure keyframe;

	//MESH BILLBOARDS
	mesh_drawable plane;
	mesh_drawable planeciel;
	mesh_drawable planesol;
	mesh_drawable planechemin;

	//CIEL
	int const static nbtaches = 50;
	int textureid[nbtaches * nbtaches * 2];
	vec3 grille[nbtaches * nbtaches * 2];
	vec3 grillevitesse[nbtaches * nbtaches * 2];
	numarray<vec3> posciel;
	numarray<vec3> rotcielx;
	numarray<vec3> rotciely;
	numarray<vec3> rotcielz;
	numarray<vec3> colciel;
	numarray<vec2> textciel;

	//SOL
	int const static nbtachessol = 50;
	int textureidsol[nbtachessol * nbtachessol * 2];
	vec3 grillesol[nbtachessol * nbtachessol * 2];
	vec3 grillevitessesol[nbtachessol * nbtachessol * 2];
	numarray<vec3> instance_positions;
	numarray<vec3> rotsolx;
	numarray<vec3> rotsoly;
	numarray<vec3> rotsolz;
	numarray<vec3> colsol;
	numarray<vec2> textsol;

	//CHEMIN
	std::vector<vec2> drop_i = { {0.8f,0.8f},{0.6f,0.4f}, {0.4f,0.1f}, {0.2f,-0.2f},  {-0.1f,-0.5f},{-0.4f,-0.6f} ,{-0.7f,-0.7f}, {-0.6f,-0.4f}, {-0.2f,0.3f}, {0,0.4f}, {0.1f,0.5f},{0.4f,0.6f}, {0.6f,0.7f}, {0.8f,0.8f} };
	int numSegments = 5;
	std::vector<vec2> splinePoints = generateCurvedSplinePoints(drop_i, numSegments, 1);
	numarray<int> chemin;
	numarray<vec3> instanceChemin;
	numarray<vec3> rotcheminx;
	numarray<vec3> rotcheminy;
	numarray<vec3> rotcheminz;
	numarray<vec2> textchemin;
	int const static nbptschemin = 100;
	numarray<int> textidchemin;
	numarray<vec3> SplineChemin;
	numarray<float> key_times;

	struct perlin_noise_parameters
	{
		float persistency = 0.0f;
		float frequency_gain = 10.0;
		int octave = 8;
		float terrain_height = 0.0f;
	};
	perlin_noise_parameters parameters;

	// Timer used for the interpolation of the position
	cgp::timer_interval timer;
	float t = 1;

	//TERRAIN
	int N_terrain_samples = 100;
	float terrain_length = 5;
	std::tuple<mesh,std::vector<vec3>> res = create_terrain_mesh(N_terrain_samples, terrain_length);
	mesh const terrain_mesh = std::get<0>(res);
	std::vector<vec3> ground_pos = std::get<1>(res);


	//FEMME
	cgp::hierarchy_mesh_drawable femme;
	mesh_drawable corps_mesh;
	mesh_drawable bras_mesh;
	mesh_drawable tete_mesh;
	mesh_drawable robe_mesh;
	mesh robe;
	int const static N = 20;
	int const static M = 10;
	vec3 robegrille[N * M];
	vec3 robevitesse[N * M];

	float ctemps=0;

	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();    // Standard initialization to be called before the animation loop
	void display_frame(); // The frame display to be called within the animation loop
	void display_gui();   // The display of the GUI, also called within the animation loop

	// Display function for semi-transparent shapes
	void display_semiTransparent();

	void mouse_move_event();
	void mouse_click_event();
	void keyboard_event();
	void idle_frame();

	void simulation_step(vec3 gr[], vec3 grv[], int lx, int ly, float dt, float k, float perturbation, float frottements, float scale, bool ciel);
	void simulation_step3D(vec3 gr[], vec3 grv[], int lx, int ly, float dt, float k, float frottements,vec3 pos);

};





