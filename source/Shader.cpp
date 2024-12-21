#include "Shader.h"
#include "Renderer.h"
#include <fstream>
#include <sstream>
#include <iostream>

Shader::Shader(const std::string& vertexFilePath, const std::string& fragmentFilePath)
    : m_VertexFilePath(vertexFilePath), m_FragmentFilePath(fragmentFilePath), m_RendererID(0)
{
    std::string vertexShader = Shader::ParseShader(vertexFilePath);
    std::string fragmentShader = Shader::ParseShader(fragmentFilePath);
    m_RendererID = Shader::CreateShader(vertexShader, fragmentShader);
}

Shader::Shader() : m_RendererID(0) {

}

Shader::~Shader()
{
    GLCall(glDeleteProgram(m_RendererID));
    std::cout << "Shader deleted" << std::endl;
}

std::string Shader::ParseShader(const std::string& filePath) {
    std::ifstream stream(filePath);
    if (!stream.is_open()) {
        std::cerr << "Error parsing shader \"" << filePath << "\". Could not open file." << std::endl;
        return "";
    }

    std::string line;
    std::stringstream ss;
    while (getline(stream, line)) {
        if (line.find("#shader") != std::string::npos || line.empty()) continue;
        ss << line << "\n";
    }
    return ss.str();
}


unsigned int Shader::CompileShader(const std::string& source, unsigned int type) {
    unsigned int id = glCreateShader(type);
    if (id == 0) {
        std::cerr << "Failed to create shader of type: " << type << std::endl;
        return 0;
    }

    const char* src = source.c_str();
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    int result;
    glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE) {
        int length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        std::vector<char> message(length);
        glGetShaderInfoLog(id, length, &length, message.data());
        std::cerr << "Failed to compile "
            << (type == GL_VERTEX_SHADER ? "Vertex" : "Fragment") << " Shader" << std::endl;
        std::cerr << message.data() << std::endl;
        glDeleteShader(id);
        return 0;
    }
    return id;
}


unsigned int Shader::CreateShader(const std::string& vertexShader, const std::string& fragmentShader) {
    if (glfwGetCurrentContext() == NULL) {
        std::cerr << "OpenGL context is not current!" << std::endl;
        return 0;
    }

    unsigned int program = glCreateProgram();
    if (program == 0) {
        std::cerr << "Failed to create OpenGL program!" << std::endl;
        return 0;
    }

    unsigned int vs = CompileShader(vertexShader, GL_VERTEX_SHADER);
    unsigned int fs = CompileShader(fragmentShader, GL_FRAGMENT_SHADER);

    if (vs == 0 || fs == 0) {
        glDeleteProgram(program);
        return 0;
    }

    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return program;
}

void Shader::AssertBound() const {
    int currentProgram;
    glGetIntegerv(GL_CURRENT_PROGRAM, &currentProgram);
    if (static_cast<unsigned int>(currentProgram) != m_RendererID) {
        std::cerr << "Warning: Uniform set on inactive shader program!" << std::endl;
    }
}

void Shader::Bind() const
{
	GLCall(glUseProgram(m_RendererID));
}

void Shader::Unbind() const
{
	GLCall(glUseProgram(0));
}

void Shader::SetUniform1i(const std::string& name, int v0)
{
    AssertBound();
    GLCall(glUniform1i(GetUniformLocation(name), v0));
}

void Shader::SetUniform1f(const std::string& name, float v0)
{
    AssertBound();
    GLCall(glUniform1f(GetUniformLocation(name), v0));
}

void Shader::SetUniform2f(const std::string& name, float v0, float v1)
{
    AssertBound();
    GLCall(glUniform2f(GetUniformLocation(name), v0, v1));
}

void Shader::SetUniform4f(const std::string& name, float v0, float v1, float v2, float v3)
{
    AssertBound();
	GLCall(glUniform4f(GetUniformLocation(name), v0, v1, v2, v3));
}

void Shader::SetUniformMat4f(const std::string& name, const glm::mat4& matrix)
{
    AssertBound();
    GLCall(glUniformMatrix4fv(GetUniformLocation(name), 1, GL_FALSE, &matrix[0][0]));
}

int Shader::GetUniformLocation(const std::string& name)
{
    if (m_UniformLocationCache.find(name) != m_UniformLocationCache.end())
        return m_UniformLocationCache[name];
    GLCall(int location = glGetUniformLocation(m_RendererID, name.c_str()));
    if (location == -1)
        std::cout << "Warning: uniform '" << name << "' doesn't exist!" << std::endl;
    m_UniformLocationCache[name] = location;
    return location;
}

void Shader::Delete() {
    Shader::~Shader();
}