#pragma once

#include <communication/commands.h>
class Commander;

class Controller
{
public:
	virtual float Control(float input, float dt) = 0;
	virtual void Command(parsed_user_command user_command, Commander &source) {}
};

