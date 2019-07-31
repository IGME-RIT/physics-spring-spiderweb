#include <iostream>

#include <stdlib.h>
#include <time.h>
// We are using the glew32s.lib
// Thus we have a define statement
// If we do not want to use the static library, we can include glew32.lib in the project properties
// If we do use the non static glew32.lib, we need to include the glew32.dll in the solution folder
// The glew32.dll can be found here $(SolutionDir)\..\External Libraries\GLEW\bin\Release\Win32

#include "Shader.h"
#include "Entity.h"


// Variables for the Height and width of the window
const GLint WIDTH = 800, HEIGHT = 800;

int maxRings = 5;
int ringEntities = 8;
float ringRadius = 0.2f;
float side = 0.02f;
bool dragging = false;
glm::vec2 mousePosition = glm::vec2();
glm::vec2 prevMousePosition = glm::vec2();

//Simple kinematic point mass
struct KinematicBody
{
	float mass = 5.0f;
	glm::vec3 position = glm::vec3();
	glm::vec3 velocity = glm::vec3();
	glm::vec3 acceleration = glm::vec3();
	glm::vec3 totalForce = glm::vec3();

	//AABB
	glm::vec2 min;
	glm::vec2 max;

	std::vector<int> springList;
	KinematicBody()
	{
		
	}

	KinematicBody(glm::vec3 position)
	{
		this->position = position;
		min = position - side;
		max = position + side;
	}

	void Update(float dt)
	{
		//Update AABB if body was just initalized
		if (totalForce == glm::vec3())
		{
			min = position - side / 2.0f;
			max = position + side / 2.0f;
			return;
		}

		acceleration = totalForce / mass;
		RK4Integration(dt);

		totalForce = glm::vec3();

		min = position - side;
		max = position + side;

	}
	//Simple Euler integration
	glm::vec3 EulerIntegration(glm::vec3 pos, glm::vec3 &velocity, float dt)
	{
		 pos += velocity * dt;

		velocity += acceleration * dt;

		return pos;
	}
	
	//4-step Runga-Kutta integration 
	void RK4Integration(float dt)
	{
		glm::vec3 k0, k1, k2, k3, k4;
		glm::vec3 vel = velocity;
		glm::vec3 pos = position;



		k0 = EulerIntegration(pos, vel, 0.0f);
		k1 = vel;

		pos += dt * k1 / 2.0f;
		k0 = EulerIntegration(pos, vel, dt/2.0f);
		k2 = vel;
		pos = position;


		pos += dt * k2 / 2.0f;
		k0 = EulerIntegration(pos, vel, dt / 2.0f);
		k3 = vel;
		pos = position;

		pos += dt * k3;
		k0 = EulerIntegration(pos, vel, dt);
		k4 = vel;
		pos = position;

		pos += dt * (k1 + (2.0f*k2) + (2.0f * k3) + k4) / 6.0f;

		velocity += acceleration * dt;

		position = pos;


	}
};
std::vector< KinematicBody*> pointMasses;
int selectedPoint = -1;

struct Spring
{
	KinematicBody* start;
	KinematicBody* end;
	glm::vec3 force;

	float k;
	float damping;
	glm::vec3 x2;
	float equilibrium;
	bool updated = false;

	Spring(KinematicBody *start, KinematicBody *end, float k, float damp)
	{
		this->start = start;
		this->end = end;
		this->k = k;
		this->damping = damp;

		equilibrium = glm::distance(start->position, end->position);
	}

	void Update()
	{
		updated = true;
		CalcForce();

		if (force == glm::vec3())
			return;

		force = glm::vec3();

		
	}

	void CalcForce()
	{
		//new X value
		float newDist = glm::distance(start->position, end->position);


		if (equilibrium == newDist)
			return;

		//force direction
		glm::vec3 direction = glm::normalize(start->position - end->position);

		//F = -k * dX - damping
		force = direction * (-k * (newDist - equilibrium) + (-damping * glm::dot(start->velocity-end->velocity, direction)));

		start->totalForce += force;
		end->totalForce -= force;
	}
};
std::vector<Spring*> springs;

//Helper method that radially updates the springs from a source body with fixed radius
void UpdateSprings(KinematicBody* source, int levels)
{
	if (levels == 0)
		return;

	for (int index : source->springList)
	{
		if (!springs[index]->updated)
			springs[index]->Update();

	}

	
	for (int index : source->springList)
	{
		if (springs[index]->start != source)
			UpdateSprings(springs[index]->start,levels-1);

		if (springs[index]->end != source)
			UpdateSprings(springs[index]->end,levels -1);
	}
	
}

//Mouse input callback methods
void mouseMoveCallback(GLFWwindow *window, GLdouble mouseX, GLdouble mouseY)
{
	prevMousePosition = mousePosition;
	mousePosition = glm::vec2(mouseX, mouseY);
	mousePosition.x = (2.0f * mousePosition.x) / WIDTH - 1.0f;
	mousePosition.y = 1.0f - (2.0f * mousePosition.y) / HEIGHT;

	
}
void mouseClickCallback(GLFWwindow *window, GLint button, GLint action, GLint mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		if (!dragging)
		{
			for (int i = 0; i < pointMasses.size() - ringEntities; i++)
			{
				if (mousePosition.x <= pointMasses[i]->max.x && mousePosition.x >= pointMasses[i]->min.x &&
					mousePosition.y <= pointMasses[i]->max.y && mousePosition.y >= pointMasses[i]->min.y)
				{
					selectedPoint = i;
					dragging = true;
					break;
				}
			}
			
		}

	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
			dragging = false;
	}

	

}

int main()
{
#pragma region OpenGL_SetUp
	//Initializes the glfw
	glfwInit();

	// Setting the required options for GLFW

	// Setting the OpenGL version, in this case 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	// Setting the Profile for the OpenGL.
	// Option of choosing between Compat and Core.
	// We choose core, because we won't be using any deprecated function from the previous versions of OpenGL
	// In other words, no backward compatibility
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Setting the forward compatibility of the application to true
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	// We don't want the window to resize as of now.
	// Therefore we will set the resizeable window hint to false.
	// To make is resizeable change the value to GL_TRUE.
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	// Create the window object
	// The first and second parameters passed in are WIDTH and HEIGHT of the window we want to create
	// The third parameter is the title of the window we want to create

	// NOTE: Fourth paramter is called monitor of type GLFWmonitor, used for the fullscreen mode.
	//		 Fifth paramter is called share of type GLFWwindow, here we can use the context of another window to create this window
	// Since we won't be using any of these two features for the current tutorial we will pass nullptr in those fields
	GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Shaders Tutorial", nullptr, nullptr);

	// We call the function glfwGetFramebufferSize to query the actual size of the window and store it in the variables.
	// This is useful for the high density screens and getting the window size when the window has resized.
	// Therefore we will be using these variables when creating the viewport for the window
	int screenWidth, screenHeight;
	glfwGetFramebufferSize(window, &screenWidth, &screenHeight);

	// Check if the window creation was successful by checking if the window object is a null pointer or not
	if (window == nullptr)
	{
		// If the window returns a null pointer, meaning the window creation was not successful
		// we print out the messsage and terminate the glfw using glfwTerminate()
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();

		// Since the application was not able to create a window we exit the program returning EXIT_FAILURE
		return EXIT_FAILURE;
	}

	// Creating a window does not make it the current context in the windows.
	// As a results if the window is not made the current context we wouldn't be able the perform the operations we want on it
	// So we make the created window to current context using the function glfwMakeContextCurrent() and passing in the Window object
	glfwMakeContextCurrent(window);

	// Enable GLEW, setting glewExperimental to true.
	// This allows GLEW take the modern approach to retrive function pointers and extensions
	glewExperimental = GL_TRUE;

	glfwSetCursorPosCallback(window, mouseMoveCallback);
	glfwSetMouseButtonCallback(window, mouseClickCallback);
	// Initialize GLEW to setup OpenGL function pointers
	if (GLEW_OK != glewInit())
	{
		// If the initalization is not successful, print out the message and exit the program with return value EXIT_FAILURE
		std::cout << "Failed to initialize GLEW" << std::endl;

		return EXIT_FAILURE;
	}

	// Setting up the viewport
	// First the parameters are used to set the top left coordinates
	// The next two parameters specify the height and the width of the viewport.
	// We use the variables screenWidth and screenHeight, in which we stored the value of width and height of the window,
	glViewport(0, 0, screenWidth, screenHeight);
#pragma endregion

#pragma region Entity_Setup

	std::vector<Entity*> entityList;
	std::vector<glm::vec2> linePoints;
	
	

	float angle = (2.0f * glm::pi<float>()) / ringEntities;
	float k = 1000;
	float damping = 10.0f;
	//Adding web center at 0,0
	entityList.push_back(new Entity());
	entityList[0]->SetVertices() = {
		// Triangle 1		
		glm::vec3(-0.5f, -0.5f, 0.0f),
		glm::vec3(0.5f, -0.5f, 0.0f),
		glm::vec3(-0.5f, 0.5f, 0.0f),
		//Triangle 2
		glm::vec3(-0.5f, 0.5f, 0.0f),
		glm::vec3(0.5f, -0.5f, 0.0f),
		glm::vec3(0.5f, 0.5f, 0.0f)
	};
	entityList[0]->transform->position = glm::vec3(0.0, 0.0, 0.0);
	entityList[0]->transform->scale = glm::vec3(side, side, 1.0f);
	entityList[0]->LoadBuffers();

	pointMasses.push_back(new KinematicBody(entityList[0]->transform->position));

	//Place the point masses of our spider web
	for (int i = 0; i < maxRings; i++)
	{
		for (int j = 0; j < ringEntities; j++)
		{
			int index = (j + i * ringEntities) + 1;

			entityList.push_back(new Entity());
			entityList[index]->SetVertices() = {
				// Triangle 1		
				glm::vec3(-0.5f, -0.5f, 0.0f),
				glm::vec3(0.5f, -0.5f, 0.0f),
				glm::vec3(-0.5f, 0.5f, 0.0f),
				//Triangle 2
				glm::vec3(-0.5f, 0.5f, 0.0f),
				glm::vec3(0.5f, -0.5f, 0.0f),
				glm::vec3(0.5f, 0.5f, 0.0f)
			};

			//We use cos and sin to place points in a circle scaled by radius
			float x = cos(angle*j)*ringRadius*(i + 1);
			float y = sin(angle*j)*ringRadius*(i + 1);

			entityList[index]->transform->position = glm::vec3(x, y, 0.0);
			entityList[index]->transform->scale = glm::vec3(side, side, 1.0f);
			entityList[index]->LoadBuffers();

			//Create a pointmass with the same position as the created entity
			pointMasses.push_back(new KinematicBody(entityList[index]->transform->position));

			
		}
	}

	Spring* spring;
	//Link springs between our point masses in each ring
	for (int i = 1; i < pointMasses.size(); i++)
	{

		if (i%ringEntities != 0)
		{
			spring = new Spring(pointMasses[i], pointMasses[i + 1], k, damping);
			pointMasses[i + 1]->springList.push_back(springs.size());
		}
		else
		{
			spring = new Spring(pointMasses[i], pointMasses[i - (ringEntities - 1)], k, damping);
			pointMasses[i - (ringEntities - 1)]->springList.push_back(springs.size());
		}
		pointMasses[i]->springList.push_back(springs.size());
		springs.push_back(spring);
		

		linePoints.push_back(glm::vec2(spring->start->position));
		linePoints.push_back(glm::vec2(spring->end->position));
	}

	//Link springs between the center and nearby points
	for (int i = 1; i <= ringEntities; i++)
	{
		spring=new Spring(pointMasses[0], pointMasses[i], k*2, damping);

		
		pointMasses[0]->springList.push_back(springs.size());
		pointMasses[i]->springList.push_back(springs.size());
		springs.push_back(spring);

		linePoints.push_back(glm::vec2(spring->start->position));
		linePoints.push_back(glm::vec2(spring->end->position));
	}

	//Link springs between our point masses across rings
	for (int i = 1; i < pointMasses.size() - ringEntities; i++)
	{

		spring = new Spring(pointMasses[i], pointMasses[i + ringEntities], k*2, damping);

		
		pointMasses[i]->springList.push_back(springs.size());
		pointMasses[i+ ringEntities]->springList.push_back(springs.size());
		springs.push_back(spring);

		linePoints.push_back(glm::vec2(spring->start->position));
		linePoints.push_back(glm::vec2(spring->end->position));

	}



#pragma endregion

	GLuint quadVAO;
	GLuint quadVBO;

	
	glGenVertexArrays(1, &quadVAO);

	glGenBuffers(1, &quadVBO);																						
	glBindVertexArray(quadVAO);																						
	glBindBuffer(GL_ARRAY_BUFFER, quadVBO);																		  

	//Defining VBO data
	glBufferData(GL_ARRAY_BUFFER,
		sizeof(glm::vec2)*linePoints.size(), 
		&linePoints[0],					 
		GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), 0);
	glBindBuffer(0, quadVBO);
	glBindVertexArray(0);
	//*/

	//Creating, initalizing and using our shader(s)
	Shader basicShader;
	basicShader.load();
	basicShader.use();
	

	// This is the game loop, the game logic and render part goes in here.
	// It checks if the created window is still open, and keeps performing the specified operations until the window is closed
	while (!glfwWindowShouldClose(window))
	{
		

		// Specifies the RGBA values which will be used by glClear to clear the color buffer
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

		// Clears contents of a screen to a preset value, previously selected, by passing in what buffer we want to clear
		// In this case we are clearing the color buffer. Thus setting a background color to the color specified previously in glClearColor
		glClear(GL_COLOR_BUFFER_BIT);

		// Calculate delta time.
		float dt = glfwGetTime();
		// Reset the timer.
		glfwSetTime(0);

		//Drag points
		if (dragging)
		{
			glm::vec3 movement = (glm::vec3(mousePosition, 0) - pointMasses[selectedPoint]->position) / 2.0f;
			glm::clamp(movement.x, -0.001f, 0.001f);
			glm::clamp(movement.y, -0.001f, 0.001f);
			movement.z = 0;
			pointMasses[selectedPoint]->position += movement;

		}
		//Clear line render list
		linePoints.clear();

		//update springs around dragged point first
		if (selectedPoint != -1)
			UpdateSprings(pointMasses[selectedPoint], 4);

		//update the other points afterwards in order of the collection
		if (selectedPoint != -1)
			for (int i = 0; i < springs.size(); i++)
			{
				if (!springs[i]->updated)
					springs[i]->Update();
			}


		//Render entities
		for (int i = 0; i < entityList.size(); i++)
		{
			if(i<entityList.size()-ringEntities)
				pointMasses[i]->Update(dt);

			entityList[i]->transform->position = pointMasses[i]->position;
			entityList[i]->Update();
			entityList[i]->RenderEntity();
		}
		//Add spring points to the render list
		for (int i = 0; i < springs.size(); i++)
		{
			linePoints.push_back(glm::vec2(springs[i]->start->position));
			linePoints.push_back(glm::vec2(springs[i]->end->position)); 
			springs[i]->updated=false;
		}
		
		//Draw the springs
		glBufferData(GL_ARRAY_BUFFER,
			sizeof(glm::vec2)*linePoints.size(),
			&linePoints[0],
			GL_STATIC_DRAW);
		glUniformMatrix4fv(2, 1, GL_FALSE, &glm::scale(glm::vec3(1, 1, 1))[0][0]);
		glBindVertexArray(quadVAO);
		glDrawArrays(GL_LINES, 0, linePoints.size());
		glBindVertexArray(0);
		

		// Swaps the front and back buffers of the specified window
		glfwSwapBuffers(window);

		// Checking for events/inputs
		glfwPollEvents();

	}

	for (int i = 0; i < entityList.size(); i++)
	{
		delete entityList[i];
	}

	for (int i = 0; i < pointMasses.size(); i++)
	{
		delete pointMasses[i];
	}

	for (int i = 0; i < springs.size(); i++)
	{
		delete springs[i];
	}
	// Terminate all the stuff related to GLFW and exit the program using the return value EXIT_SUCCESS
	glfwTerminate();

	return EXIT_SUCCESS;
}