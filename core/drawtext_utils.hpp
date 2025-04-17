#pragma once
#include <ctime>
#include <iomanip>
#include <sstream>
#include <regex>
#include <cstdlib>

inline std::string expand_text_template(const std::string &input) {
	std::string result = input;

	// 1. Handle %{localtime:<format>}
	std::regex time_expr(R"%(%\{localtime:([^}]+)\})%");
	std::smatch match;
	while (std::regex_search(result, match, time_expr)) {
		std::string fmt = match[1];
		std::time_t t = std::time(nullptr);
		std::tm tm;
		localtime_r(&t, &tm);
		std::ostringstream oss;
		oss << std::put_time(&tm, fmt.c_str());
		result.replace(match.position(0), match.length(0), oss.str());
	}

	// 2. Handle ${ENV_VAR}
	std::regex env_expr(R"(\$\{([^\}]+)\})");
	while (std::regex_search(result, match, env_expr)) {
		const char* val = std::getenv(match[1].str().c_str());
		result.replace(match.position(0), match.length(0), val ? val : "");
	}

	return result;
}
