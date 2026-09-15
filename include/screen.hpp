#pragma once
#include <string>
class Screen{
	private:
		std::string text;
	public:
		void print(std::string text);
		void redrawscreen();

};
