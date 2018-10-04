#pragma once

#include <Libs/json/json.h>

#include <Modules/Configuration/Configuration.h>
#include "Tools/Storage/UniValue/UniValue2Json.hpp"
#include <Tools/Storage/UniValue/UniValue.h>

#include <boost/signals2.hpp>
#include <list>
#include <map>
#include <memory>
#include <stdexcept>

namespace MiniConfigHandle
{

bool mountFile(const std::string &filename, Uni::Value &val)
{
    Json::Reader reader;

    std::ifstream stream(filename);
    if (!stream.is_open())
    {
        return false;
    }
    try
    {
        Json::Value tmp;
        reader.parse(stream, tmp);
        val = Uni::Converter::toUniValue(tmp);
    }
    catch (std::exception &exc)
    {
        throw ConfigurationException(exc.what(), ConfigurationException::ERROR_UNKNOWN);
    }
    return true;
}

} // namespace MiniConfigHandle