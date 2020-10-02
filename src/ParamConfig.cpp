/**
 * @file    ParamConfig.cpp
 *
 * @author  btran
 *
 */

#include <sensors_calib/utils/ParamConfig.hpp>

namespace perception
{
template <> std::string getValueAs<std::string>(const rapidjson::Document& rapidjsonDoc, const std::string& key)
{
    const rapidjson::Value& node = rapidjsonDoc[key.c_str()];
    if (node.IsNull()) {
        throw std::runtime_error(key + " does not exist");
    }

    if (!node.IsString()) {
        throw std::runtime_error("mismatch data type: " + key);
    }

    return node.GetString();
}

}  // namespace perception
