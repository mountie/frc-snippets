#include <iostream>

#include <OpenNI.h>


void die (void)
{
	std::cerr << "ERROR: " << openni::OpenNI::getExtendedError() << std::endl;
}

int main (int argc, char* argv[])
{
	openni::Status s;
	openni::OpenNI::initialize();

	openni::Device d;
	s = d.open(openni::ANY_DEVICE);

	if (s)
		die();

	d.close();

	openni::OpenNI::shutdown();
}
