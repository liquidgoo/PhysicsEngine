#include <glad/gl.h>  // Must be first
#include <GLFW/glfw3.h>
#include <iostream>

int main() {
    std::cout<<"Starting Physics Debug Application" << std::endl;
    if (!glfwInit()) return -1;
    
    std::cout<<"GLFW initialized successfully" << std::endl;
    GLFWwindow* window = glfwCreateWindow(800, 600, "Physics Debug", NULL, NULL);
    if (!window) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLAD after setting the context
    if (!gladLoadGL(glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
