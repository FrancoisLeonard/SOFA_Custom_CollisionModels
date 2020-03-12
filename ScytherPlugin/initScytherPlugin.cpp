#include "scyther.h"

#include <cstring>
#include <string>

namespace sofa
{

namespace component
{

extern "C" {
    SOFA_SCYTHER_API void initExternalModule();
    SOFA_SCYTHER_API const char* getModuleName();
    SOFA_SCYTHER_API const char* getModuleVersion();
    SOFA_SCYTHER_API const char* getModuleLicense();
    SOFA_SCYTHER_API const char* getModuleDescription();
    SOFA_SCYTHER_API const char* getModuleComponentList();
}

void initExternalModule()
{
	static bool first = true;
	if (first)
	{
		first = false;
	}
}

const char* getModuleName()
{
    return "ScytherPlugin";
}

const char* getModuleVersion()
{
	return "beta 1.0";
}

const char* getModuleDescription()
{
    return "Sofa Scyther plugin";
}

const char* getModuleComponentList()
{
	std::string commonentlist;

    commonentlist += "";
	commonentlist += "";

	return commonentlist.c_str();
}
const char* getModuleLicense()
{
    return "François Leonard";
}

} // namespace component

} // namespace sofa

