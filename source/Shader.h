#pragma once

#include <string>
#include <unordered_map>
#include <glm/glm.hpp>

class Shader
{
private:
	unsigned int m_RendererID;
	std::string m_VertexFilePath;
	std::string m_FragmentFilePath;

	std::unordered_map<std::string, int> m_UniformLocationCache;
public:
	Shader();
	Shader(const std::string& vertexFilePath, const std::string& fragmentFilePath);
	~Shader();

	void Bind() const;
	void Unbind() const;

	void AssertBound() const;

	void SetUniform1i(const std::string& name, int v0);
	void SetUniform1f(const std::string& name, float v0);
	void SetUniform2f(const std::string& name, float v0, float v1);
	void SetUniform4f(const std::string& name, float v0, float v1, float v2, float v3);
	void SetUniformMat4f(const std::string& name, const glm::mat4& matrix);
	void Delete();
private:
	std::string ParseShader(const std::string& filePath);
	unsigned int CompileShader(const std::string& source, unsigned int type);
	unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
	int GetUniformLocation(const std::string& name);
};