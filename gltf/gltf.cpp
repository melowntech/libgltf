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

#include "utility/base64.hpp"
#include "utility/streams.hpp"
#include "utility/format.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "gltf.hpp"
#include "detail.hpp"

namespace fs = boost::filesystem;
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
    value = utility::format("%d.%d", version.major, version.minor);
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
    build(value, "name", bufferView.name);
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
    build(value, "name", accessor.name);
}

void build(Json::Value &value, const Scene &scene)
{
    namedCommon(value, scene);
    build(value, "name", scene.name);
    build(value, "nodes", scene.nodes);
}

void build(Json::Value &value, const Node &node)
{
    namedCommon(value, node);
    build(value, "name", node.name);
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
    build(value, "name", mesh.name);
    build(value, "primitives", mesh.primitives);
}

void build(Json::Value &value, const Sampler &sampler)
{
    namedCommon(value, sampler);
    build(value, "magFilter", sampler.magFilter);
    build(value, "minFilter", sampler.minFilter);
    build(value, "wrapS", sampler.wrapS);
    build(value, "wrapT", sampler.wrapT);
    build(value, "name", sampler.name);
}

void build(Json::Value &value, const Texture &texture)
{
    namedCommon(value, texture);
    value["sampler"] = texture.sampler;
    value["source"] = texture.source;
    build(value, "name", texture.name);
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
    build(value, "name", material.name);
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

void parse(std::vector<std::string> &list, const Json::Value &value
           , const char *member)
{
    if (!value.isMember(member)) { return; }
    parse(list, value["member"]);
}

void parse(Model &model, const Json::Value &value)
{
    common(model, value);

#if 0
    build(value["asset"], model.asset);
    build(value, "scenes", model.scenes);
#endif
    Json::get(model.scene, value, "scene");
#if 0
    build(value, "nodes", model.nodes);
    build(value, "meshes", model.meshes);
    build(value, "samplers", model.samplers);
    build(value, "images", model.images);
    build(value, "textures", model.textures);
    build(value, "materials", model.materials);
    build(value, "buffers", model.buffers);
    build(value, "bufferViews", model.bufferViews);
    build(value, "accessors", model.accessors);
#endif

    parse(model.extensionsUsed, value, "extensionsUsed");
    parse(model.extensionsRequired, value, "extensionsRequired");
}

} // namespace detail

void read(Model &model, const Json::Value &content, const fs::path &path)
{
    detail::parse(model, content);
    (void) path;
}

} // namespace gltf
