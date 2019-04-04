/**
 * Copyright (c) 2019 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/as.hpp"

#include "../io.hpp"
#include "support.hpp"

namespace gltf { namespace detail {

void parse(std::vector<std::string> &list, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Expected JSON array.";
    }

    list.reserve(value.size());

    for (const auto &item : value) {
        if (!item.isString()) {
            LOGTHROW(err1, Json::Error)
                << "Expected JSON array of strings.";
        }
        list.push_back(item.asString());
    }
}

void parse(Indices &indices, const Json::Value &value)
{
    if (value.type() != Json::arrayValue) {
        LOGTHROW(err1, Json::Error)
            << "Indices are not an array.";
    }

    indices.clear();
    indices.reserve(value.size());
    for (const auto &item : value) {
        indices.push_back(item.asInt());
    }
}

void common(CommonBase &cb, const Json::Value &value)
{
    if (value.isMember("extensions")) {
        const auto &extensions(value["extensions"]);
        for (const auto &name : extensions.getMemberNames()) {
            cb.extensions.insert(Extensions::value_type
                                 (name, extensions[name]));
        }
    }
    if (value.isMember("extras")) {
        cb.extras = value["extras"];
    }
}

void namedCommon(NamedCommonBase &ncb, const Json::Value &value)
{
    common(ncb, value);
    Json::get(ncb.name, value, "name");
}

void parse(Version &version, const Json::Value &value)
{
    std::string raw;
    Json::get(raw, value);
    version = boost::lexical_cast<Version>(raw);
}

void parse(math::Point3d &point, const Json::Value &value)
{
    if ((value.type() != Json::arrayValue) || (value.size() != 3)) {
        LOGTHROW(err1, Json::Error)
            << "3D point doesn'tt have 3 elements (but"
            << value.size() << ").";
    }

    point(0) = value[0].asDouble();
    point(1) = value[1].asDouble();
    point(2) = value[2].asDouble();
}

void parse(math::Point4d &point, const Json::Value &value)
{
    if ((value.type() != Json::arrayValue) || (value.size() != 4)) {
        LOGTHROW(err1, Json::Error)
            << "4D point doesn'tt have 4 elements (but"
            << value.size() << ").";
    }

    point(0) = value[0].asDouble();
    point(1) = value[1].asDouble();
    point(2) = value[2].asDouble();
    point(3) = value[3].asDouble();
}

void parse(math::Matrix4 &matrix, const Json::Value &value)
{
    if ((value.type() != Json::arrayValue) || (value.size() != 16)) {
        LOGTHROW(err1, Json::Error)
            << "Transformation matrix doesn't have 16 elements (but"
            << value.size() << ").";
    }

    matrix = boost::numeric::ublas::zero_matrix<double>(4, 4);

    // read matrix as a column major order
    for (int i(0), column(0); column < 4; ++column) {
        for (int row(0); row < 4; ++row, ++i) {
            matrix(row, column) = value[i].asDouble();
        }
    }
}

void parse(ComponentValue &cv, const Json::Value &value)
{
    switch (value.type()) {
    case Json::intValue: case Json::uintValue:
        cv = value.asInt();
        break;

    case Json::realValue:
        cv = value.asDouble();
        break;

    default:
        LOGTHROW(err1, Json::Error)
            << "ComponentValue must be a number.";
        break;
    }
}

void parse(ComponentType &ct, const Json::Value &value)
{
    Json::check(value, Json::intValue, "ComponentType must be an integer.");
    switch ((ct = static_cast<ComponentType>(value.asInt()))) {
    case ComponentType::byte: case ComponentType::ubyte:
    case ComponentType::short_: case ComponentType::ushort:
    case ComponentType::uint: case ComponentType::float_:
        break;

    default:
        LOGTHROW(err1, Json::Error)
            << "Invalid numeric value of ComponentType ("
            << value.asInt() << ").";
    }
}

void parse(WrappingMode &wm, const Json::Value &value)
{
    Json::check(value, Json::intValue, "WrappingMode must be an integer.");
    switch ((wm = static_cast<WrappingMode>(value.asInt()))) {
    case WrappingMode::repeat: case WrappingMode::clamp: break;
    case WrappingMode::mirrored: break;
    default:
        LOGTHROW(err1, Json::Error)
            << "Invalid numeric value of Target (" << value.asInt() << ").";
    }
}

void parse(Target &target, const Json::Value &value)
{
    Json::check(value, Json::intValue, "Target must be an integer.");
    switch ((target = static_cast<Target>(value.asInt()))) {
    case Target::arrayBuffer: case Target::elementArrayBuffer: break;
    default:
        LOGTHROW(err1, Json::Error)
            << "Invalid numeric value of Target (" << value.asInt() << ").";
    }
}

void parse(PrimitiveMode &mode, const Json::Value &value)
{
    Json::check(value, Json::intValue, "PromitiveMode must be an integer.");
    switch ((mode = static_cast<PrimitiveMode>(value.asInt()))) {
    case PrimitiveMode::points: case PrimitiveMode::lines:
    case PrimitiveMode::lineLoop:  case PrimitiveMode::lineStrip:
    case PrimitiveMode::triangles: case PrimitiveMode::triangleStrip:
    case PrimitiveMode::triangleFan:
        break;
    default:
        LOGTHROW(err1, Json::Error)
            << "Invalid numeric value of PrimitiveMode ("
            << value.asInt() << ").";
    }
}

} } // namespace gltf::detail
