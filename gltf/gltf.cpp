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
#include <boost/utility/in_place_factory.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "utility/base64.hpp"
#include "utility/uri.hpp"
#include "utility/streams.hpp"
#include "utility/format.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "gltf.hpp"
#include "io.hpp"
#include "detail.hpp"

namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace ublas = boost::numeric::ublas;

namespace gltf {

namespace detail {

template <typename T>
typename std::enable_if<!std::is_enum<T>::value, void>::type
build(Json::Value &object, const T &value);

template <typename T>
void build(Json::Value &object, const char *name
           , const boost::optional<T> &value);

template <typename T>
void build(Json::Value &value, const std::vector<T> &list);

template <typename T>
void build(Json::Value &object, const char *name, const std::vector<T> &list);

template <typename T>
void build(Json::Value &object, const char *name
           , const std::map<std::string, T> &map);

/** Any is expected to be Json::Value instance.
 */
void build(Json::Value &object, const char *name, const boost::any &value)
{
    if (const auto *valueObject = boost::any_cast<Json::Value>(&value)) {
        object[name] = *valueObject;
    }
}

void build(Json::Value &object, const math::Matrix4 &matrix)
{
    // dump matrix as a column major
    object = Json::arrayValue;
    for (int column(0); column < 4; ++column) {
        for (int row(0); row < 4; ++row) {
            object.append(matrix(row, column));
        }
    }
}

void build(Json::Value &object, const math::Point2d &p)
{
    object = Json::arrayValue;
    object.append(p(0));
    object.append(p(1));
}

void build(Json::Value &object, const math::Point3d &p)
{
    object = Json::arrayValue;
    object.append(p(0));
    object.append(p(1));
    object.append(p(2));
}

void build(Json::Value &object, const math::Point4d &p)
{
    object = Json::arrayValue;
    object.append(p(0));
    object.append(p(1));
    object.append(p(2));
    object.append(p(3));
}

void build(Json::Value &value, const char *name, const Indices &indices)
{
    if (indices.empty()) { return; }
    auto &list(value[name] = Json::arrayValue);
    for (auto index : indices) { list.append(index); }
}

void build(Json::Value &value, const Version &version)
{
    value = boost::lexical_cast<std::string>(version);
}

void common(Json::Value &value, const CommonBase &cb)
{
    value = Json::objectValue;
    build(value, "extensions", cb.extensions);
    build(value, "extras", cb.extras);
}

void namedCommon(Json::Value &value, const NamedCommonBase &ncb)
{
    common(value, ncb);
    build(value, "name", ncb.name);
}

void build(Json::Value &value, const Asset &asset)
{
    common(value, asset);
    build(value, "copyright", asset.copyright);
    build(value, "generator", asset.generator);
    build(value["version"], asset.version);
    build(value, "minVersion", asset.minVersion);
}


void build(Json::Value &value, const Buffer &buffer)
{
    struct BuildBuffer : public boost::static_visitor<void> {
        Json::Value &value;
        BuildBuffer(Json::Value &value) : value(value) {}

        void operator()(const InlineBuffer &buffer) {
            namedCommon(value, buffer);
            value["uri"]
                = utility::concat("data:application/octet-stream;base64,"
                                  , utility::base64::encode
                                  (buffer.data.data(), buffer.data.size()));
            build(value["byteLength"], Json::UInt64(buffer.data.size()));
        }

        void operator()(const ExternalBuffer &buffer) {
            namedCommon(value, buffer);
            build(value, "uri", buffer.uri);
            build(value["byteLength"], Json::UInt64(buffer.byteLength));
        }
    } bi(value);
    boost::apply_visitor(bi, buffer);
}

void build(Json::Value &value, const BufferView &bufferView)
{
    namedCommon(value, bufferView);
    build(value["buffer"], bufferView.buffer);
    build(value, "byteOffset", bufferView.byteOffset);
    build(value["byteLength"], Json::UInt64(bufferView.byteLength));
    build(value, "byteStride", bufferView.byteStride);
    build(value, "target", bufferView.target);
}

void build(Json::Value &value, const ComponentValue &cv)
{
    struct BuildValue : public boost::static_visitor<void> {
        Json::Value &value;
        BuildValue(Json::Value &value) : value(value) {}

        void operator()(const int &v) { value = v; }
        void operator()(const double &v) { value = v; }
    } bv(value);
    boost::apply_visitor(bv, cv);
}

void build(Json::Value &value, const Accessor &accessor)
{
    namedCommon(value, accessor);
    build(value, "bufferView", accessor.bufferView);
    build(value, "offset", accessor.offset);
    build(value["componentType"], static_cast<int>(accessor.componentType));
    build(value, "normalized", accessor.normalized);
    build(value["count"], Json::UInt64(accessor.count));
    build(value["type"], boost::lexical_cast<std::string>(accessor.type));
    build(value, "max", accessor.max);
    build(value, "min", accessor.min);
}

void build(Json::Value &value, const Scene &scene)
{
    namedCommon(value, scene);
    build(value, "nodes", scene.nodes);
}

void build(Json::Value &value, const Node &node)
{
    namedCommon(value, node);
    build(value, "camera", node.camera);
    build(value, "children", node.children);
    build(value, "matrix", node.matrix);
    build(value, "rotation", node.rotation);
    build(value, "scale", node.scale);
    build(value, "translation", node.translation);
    build(value, "mesh", node.mesh);
}

void build(Json::Value &value, const Primitive &primitive)
{
    common(value, primitive);
    auto &attributes(value["attributes"] = Json::objectValue);
    for (const auto &pair : primitive.attributes) {
        attributes[boost::lexical_cast<std::string>(pair.first)] = pair.second;
    }
    build(value, "indices", primitive.indices);
    build(value, "material", primitive.material);
    build(value, "mode", primitive.mode);
    build(value, "targets", primitive.targets);
}

void build(Json::Value &value, const Mesh &mesh)
{
    namedCommon(value, mesh);
    build(value, "primitives", mesh.primitives);
}

void build(Json::Value &value, const Sampler &sampler)
{
    namedCommon(value, sampler);
    build(value, "magFilter", sampler.magFilter);
    build(value, "minFilter", sampler.minFilter);
    build(value, "wrapS", sampler.wrapS);
    build(value, "wrapT", sampler.wrapT);
}

void build(Json::Value &value, const Texture &texture)
{
    namedCommon(value, texture);
    value["sampler"] = texture.sampler;
    value["source"] = texture.source;
}

void build(Json::Value &value, const Image &image)
{
    struct BuildImage : public boost::static_visitor<void> {
        Json::Value &value;
        BuildImage(Json::Value &value) : value(value) {}

        void operator()(const InlineImage &image) {
            namedCommon(value, image);
            value["uri"]
                = utility::concat("data:", image.mimeType, ";base64,"
                                  , utility::base64::encode
                                  (image.data.data(), image.data.size()));
        }

        void operator()(const ExternalImage &image) {
            namedCommon(value, image);
            value["uri"] = image.uri;
        }

        void operator()(const BufferViewImage &image) {
            namedCommon(value, image);
            value["bufferView"] = image.bufferView;
            value["mimeType"] = image.mimeType;
        }
    } bi(value);
    boost::apply_visitor(bi, image);
}

void build(Json::Value &value, const TextureInfo &textureInfo)
{
    common(value, textureInfo);
    build(value["index"], textureInfo.index);
    build(value, "texCoord", textureInfo.texCoord);
    build(value, "scale", textureInfo.scale);
}

void build(Json::Value &value
           , const PbrMetallicRoughness &pbrMetallicRoughness)
{
    common(value, pbrMetallicRoughness);
    build(value, "baseColorTexture", pbrMetallicRoughness.baseColorTexture);
    build(value, "metallicFactor", pbrMetallicRoughness.metallicFactor);
    build(value, "roughnessFactor", pbrMetallicRoughness.roughnessFactor);
}

void build(Json::Value &value, const Material &material)
{
    namedCommon(value, material);
    build(value, "pbrMetallicRoughness", material.pbrMetallicRoughness);
}

template<class T>
typename std::enable_if<std::is_enum<T>::value, void>::type
build(Json::Value &object, const T &value)
{
    object = static_cast<int>(value);
}

template <typename T>
typename std::enable_if<!std::is_enum<T>::value, void>::type
build(Json::Value &object, const T &value)
{
    object = value;
}

template <>
void build<std::size_t>(Json::Value &object, const std::size_t &value)
{
    object = Json::UInt64(value);
}

template <typename T>
void build(Json::Value &object, const char *name
           , const boost::optional<T> &value)
{
    if (value) { build(object[name], *value); }
}

template <typename T>
void build(Json::Value &value, const std::vector<T> &list)
{
    value = Json::arrayValue;
    for (const auto &element : list) {
        build(value.append({}), element);
    }
}

template <typename T>
void build(Json::Value &object, const char *name, const std::vector<T> &list)
{
    if (list.empty()) { return; }

    auto &value(object[name] = Json::arrayValue);
    for (const auto &element : list) { build(value.append({}), element); }
}

template <typename T>
void build(Json::Value &object, const char *name
           , const std::map<std::string, T> &map)
{
    if (map.empty()) { return; }

    auto &value(object[name] = Json::objectValue);
    for (const auto &element : map) {
        build(value, element.first.c_str(), element.second);
    }
}

void build(Json::Value &value, const Model &model)
{
    common(value, model);

    build(value["asset"], model.asset);
    build(value, "scenes", model.scenes);
    build(value, "scene", model.scene);
    build(value, "nodes", model.nodes);
    build(value, "meshes", model.meshes);
    build(value, "samplers", model.samplers);
    build(value, "images", model.images);
    build(value, "textures", model.textures);
    build(value, "materials", model.materials);
    build(value, "buffers", model.buffers);
    build(value, "bufferViews", model.bufferViews);
    build(value, "accessors", model.accessors);

    // extensions; TODO: dynamic?
    build(value, "extensionsUsed", model.extensionsUsed);
    build(value, "extensionsRequired", model.extensionsRequired);
}

} // namespace detail

boost::any emptyObject()
{
    return Json::Value(Json::objectValue);
}

math::Matrix4 zup2yup()
{
    math::Matrix4 m(ublas::zero_matrix<double>(4, 4));
    m(0, 0) = 1.0;
    m(1, 2) = 1.0;
    m(2, 1) = -1.0;
    m(3, 3) = 1.0;
    return m;
}

math::Matrix4 yup2zup()
{
    math::Matrix4 m(ublas::zero_matrix<double>(4, 4));
    m(0, 0) = 1.0;
    m(1, 2) = -1.0;
    m(2, 1) = 1.0;
    m(3, 3) = 1.0;
    return m;
}

void write(std::ostream &os, const Model &model)
{
    Json::Value value;
    detail::build(value, model);
    Json::write(os, value, false);
}

void write(const fs::path &path, const Model &model)
{
    LOG(info1) << "Saving model to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    write(f, model);
    f.close();
}

namespace detail {

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

template <typename T>
void parse(boost::optional<T> &dst, const Json::Value &value
           , const char *member);

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const char *member);

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value);

void parse(std::vector<std::string> &list, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Expected JSON array.";
    }

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

    indices.reserve(value.size());
    for (const auto &item : value) {
        indices.push_back(item.asInt());
    }
}

void parse(Version &version, const Json::Value &value)
{
    std::string raw;
    Json::get(raw, value);
    version = boost::lexical_cast<Version>(raw);
}

void parse(Asset &asset, const Json::Value &value)
{
    common(asset, value);
    Json::get(asset.copyright, value, "copyright");
    Json::get(asset.generator, value, "generator");
    parse(asset.version, value["version"]);
    parse(asset.minVersion, value, "minVersion");
}


void parse(Scene &scene, const Json::Value &value)
{
    namedCommon(scene, value);
    parse(scene.nodes, value, "nodes");
}

void parse(Sampler &sampler, const Json::Value &value)
{
    namedCommon(sampler, value);
    Json::get(sampler.magFilter, value, "magFilter");
    Json::get(sampler.minFilter, value, "minFilter");
    Json::get(sampler.wrapS, value, "wrapS");
    Json::get(sampler.wrapT, value, "wrapT");
}

void parse(Texture &texture, const Json::Value &value)
{
    namedCommon(texture, value);
    Json::get(texture.sampler, value, "sampler");
    Json::get(texture.source, value, "source");
}

void parse(TextureInfo &textureInfo, const Json::Value &value)
{
    common(textureInfo, value);
    Json::get(textureInfo.index, value, "index");
    Json::get(textureInfo.texCoord, value, "texCoord");
    Json::get(textureInfo.scale, value, "scale");
}

void parse(PbrMetallicRoughness &pbrMetallicRoughness
           , const Json::Value &value)
{
    common(pbrMetallicRoughness, value);
    parse(pbrMetallicRoughness.baseColorTexture, value, "baseColorTexture");
    Json::get(pbrMetallicRoughness.metallicFactor, value, "metallicFactor");
    Json::get(pbrMetallicRoughness.roughnessFactor, value, "roughnessFactor");
}

void parse(Material &material, const Json::Value &value)
{
    namedCommon(material, value);
    parse(material.pbrMetallicRoughness, value, "pbrMetallicRoughness");
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

void parse(BufferView &bufferView, const Json::Value &value)
{
    namedCommon(bufferView, value);

    Json::get(bufferView.buffer, value, "buffer");
    Json::get(bufferView.byteOffset, value, "byteOffset");
    Json::get(bufferView.byteLength, value, "byteLength");
    Json::get(bufferView.byteStride, value, "byteStride");
    parse(bufferView.target, value, "target");
}

void parse(Accessor &accessor, const Json::Value &value)
{
    namedCommon(accessor, value);
    Json::get(accessor.bufferView, value, "bufferView");
    Json::get(accessor.offset, value, "offset");
    parse(accessor.componentType, value["componentType"]);
    Json::get(accessor.normalized, value, "normalized");
    Json::get(accessor.count, value, "count");
    Json::get(accessor.type, value, "type");
    parse(accessor.max, value, "max");
    parse(accessor.min, value, "min");
}

void parse(math::Matrix4 &matrix, const Json::Value &value)
{
    if ((value.type() != Json::arrayValue) || (value.size() != 16)) {
        LOGTHROW(err1, Json::Error)
            << "Transformation matrix doesn't have 16 elements (but"
            << value.size() << ").";
    }

    // read matrix as a column major order
    for (int i(0), column(0); column < 4; ++column) {
        for (int row(0); row < 4; ++row, ++i) {
            matrix(row, column) = value[i].asDouble();
        }
    }
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

void parse(Node &node, const Json::Value &value)
{
    namedCommon(node, value);
    Json::get(node.camera, value, "camera");
    parse(node.children, value, "children");
    parse(node.matrix, value, "matrix");
    parse(node.rotation, value, "rotation");
    parse(node.scale, value, "scale");
    parse(node.translation, value, "translation");
    Json::get(node.mesh, value, "mesh");
}

void parse(Primitive &primitive, const Json::Value &value)
{
    common(primitive, value);
    const auto &attributes(value["attributes"]);
    for (const auto &name : attributes.getMemberNames()) {
        Index i;
        Json::get(i, attributes, name.c_str());
        primitive.attributes.insert
            (Primitive::Attributes::value_type
             (boost::lexical_cast<AttributeSemantic>(name)
              , i));
    }

    Json::get(primitive.indices, value, "indices");
    Json::get(primitive.material, value, "material");
    parse(primitive.mode, value, "mode");
    parse(primitive.targets, value, "targets");
}

void parse(Mesh &mesh, const Json::Value &value)
{
    namedCommon(mesh, value);
    parse(mesh.primitives, value, "primitives");
}

void parseDataUri(const std::string &uri, Data &data, std::string &mimeType)
{
    if (!ba::istarts_with(uri, "data:")) {
        LOGTHROW(err1, Json::Error)
            << "Invalid data: URI: no data: scheme found.";
    }

    const auto comma(uri.find(',', 5));
    if (comma == std::string::npos) {
        LOGTHROW(err1, Json::Error)
            << "No comma in data: URI.";
    }

    bool base64(false);
    // find last semicolon from comma and match the segment base64 marker
    const auto semi(uri.rfind(';', comma));
    if (semi != std::string::npos) {
        base64 = !uri.compare(semi, comma - semi, ";base64");
    }
    if (base64) {
        mimeType = uri.substr(5, semi - 5);
    } else {
        mimeType = uri.substr(5, comma - 5);
    }

    const auto decoded
        (base64
         ? utility::base64::decode(uri.begin() + comma + 1, uri.end())
         : utility::urlDecode(uri.begin() + comma + 1, uri.end()));
    data.assign(decoded.begin(), decoded.end());
}

void parse(Buffer &buffer, const Json::Value &value)
{
    // buffer-view-image?
    OptString uri;
    Json::get(uri, value, "uri");
    if (uri && ba::istarts_with(*uri, "data:")) {
        // inline buffer
        auto &b(boost::get<InlineBuffer>(buffer = InlineBuffer()));
        namedCommon(b, value);
        std::size_t byteLength;
        Json::get(byteLength, value, "byteLength");
        std::string mimeType; // ignored
        parseDataUri(*uri, b.data, mimeType);
        if (byteLength != b.data.size()) {
            LOGTHROW(err1, Json::Error)
                << "Mismatching buffers byteLength: byteLength="
                << byteLength << ", data uri data size="
                << b.data.size();
        }
        return;
    }

    // external buffer (with or without uri)
    auto &b(boost::get<ExternalBuffer>(buffer = ExternalBuffer()));
    namedCommon(b, value);
    Json::get(b.byteLength, value, "byteLength");
    b.uri = uri;
}

void parse(Image &image, const Json::Value &value)
{
    // buffer-view-image?
    if (value.isMember("bufferView")) {
        auto &i(boost::get<BufferViewImage>(image = BufferViewImage()));
        namedCommon(i, value);
        Json::get(i.bufferView, value, "bufferView");
        Json::get(i.mimeType, value, "mimeType");
        return;
    }

    std::string uri;
    Json::get(uri, value, "uri");
    if (ba::istarts_with(uri, "data:")) {
        // inline image, parse uri
        auto &i(boost::get<InlineImage>(image = InlineImage()));
        namedCommon(i, value);
        parseDataUri(uri, i.data, i.mimeType);
        return;
    }

    // external image
    auto &i(boost::get<ExternalImage>(image = ExternalImage()));
    namedCommon(i, value);
    i.uri = uri;
}

void parse(Model &model, const Json::Value &value)
{
    common(model, value);

    parse(model.asset, value["asset"]);
    parse(model.scenes, value, "scenes");
    parse(model.samplers, value, "samplers");
    Json::get(model.scene, value, "scene");

    parse(model.nodes, value, "nodes");
    parse(model.meshes, value, "meshes");
    parse(model.samplers, value, "samplers");
    parse(model.images, value, "images");
    parse(model.textures, value, "textures");
    parse(model.materials, value, "materials");
    parse(model.buffers, value, "buffers");
    parse(model.bufferViews, value, "bufferViews");
    parse(model.accessors, value, "accessors");

    parse(model.extensionsUsed, value, "extensionsUsed");
    parse(model.extensionsRequired, value, "extensionsRequired");
}

template <typename T>
void parse(boost::optional<T> &dst, const Json::Value &value
           , const char *member)
{
    if (value.isMember(member)) {
        parse(*(dst = boost::in_place()), value[member]);
    }
}

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const char *member)
{
    if (value.isMember(member)) {
        parse(dst, value[member]);
    }
}

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value)
{
    if (value.type() != Json::arrayValue) {
        LOGTHROW(err1, Json::Error)
            << "Expected an array.";
    }

    dst.resize(value.size());
    auto idst(dst.begin());
    for (const auto &item : value) {
        parse(*idst++, item);
    }
}

} // namespace detail

void read(Model &model, const Json::Value &content, const fs::path &path
          , int version)
{
    switch (version) {
    case 1:
        LOGTHROW(err1, Json::Error)
            << "Parsing of glTF version 1 not supported (" << path << ").";
        break;

    case 2: break;
    default:
        if (version > 0) {
            LOGTHROW(err1, Json::Error)
                << "Parsing of glTF version " << version << " not supported("
                << path << ")";
        }
    }

    detail::parse(model, content);
}

} // namespace gltf
