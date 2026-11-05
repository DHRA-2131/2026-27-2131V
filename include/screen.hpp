#pragma once
#include <string>
class Screen{
	private:
		std::string text;
		int line = 1;
	public:
		void print(std::string text);
		void redraw();
		void clear();

};
