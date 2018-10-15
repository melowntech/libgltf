#include <boost/lexical_cast.hpp>

#include "utility/base64.hpp"
#include "utility/streams.hpp"
#include "utility/format.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "./gltf.hpp"

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

Json::Value& extensions(Json::Value &object)
{
    if (object.isMember("extensions")) {
        return object["extensions"];
    }
    return object["extensions"] = Json::objectValue;
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

void build(Json::Value &value, const Asset &asset)
{
    build(value, "copyright", asset.copyright);
    build(value, "generator", asset.generator);
    build(value["version"], asset.version);
    build(value, "minVersion", asset.minVersion);
}

void build(Json::Value &value, const Buffer &buffer)
{
    value = Json::objectValue;
    build(value, "uri", buffer.uri);
    build(value["byteLength"], Json::UInt64(buffer.byteLength));
    build(value, "name", buffer.name);
}

void build(Json::Value &value, const BufferView &bufferView)
{
    value = Json::objectValue;
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
    value = Json::objectValue;
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
    value = Json::objectValue;
    build(value, "name", scene.name);
    build(value, "nodes", scene.nodes);
}

void build(Json::Value &value, const Node &node)
{
    value = Json::objectValue;
    build(value, "name", node.name);
    build(value, "camera", node.camera);
    build(value, "children", node.children);
    build(value, "matrix", node.matrix);
    build(value, "mesh", node.mesh);
}

void build(Json::Value &value, const Primitive &primitive)
{
    value = Json::objectValue;
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
    value = Json::objectValue;
    build(value, "name", mesh.name);
    build(value, "primitives", mesh.primitives);
}

void build(Json::Value &value, const Sampler &sampler)
{
    value = Json::objectValue;
    build(value, "magFilter", sampler.magFilter);
    build(value, "minFilter", sampler.minFilter);
    build(value, "wrapS", sampler.wrapS);
    build(value, "wrapT", sampler.wrapT);
    build(value, "name", sampler.name);
}

void build(Json::Value &value, const Texture &texture)
{
    value = Json::objectValue;
    value["sampler"] = texture.sampler;
    value["source"] = texture.source;
    build(value, "name", texture.name);
}

void build(Json::Value &value, const Image &image)
{
    struct BuildImage : public boost::static_visitor<void> {
        Json::Value &value;
        BuildImage(Json::Value &value) : value(value = Json::objectValue) {}

        void operator()(const InlineImage &image) {
            value["uri"]
                = utility::concat("data:", image.mimeType, ";base64,"
                                  , utility::base64::encode
                                  (image.data.data(), image.data.size()));
        }

        void operator()(const ReferencedImage &image) {
            value["uri"] = image.uri;
        }

        void operator()(const BufferViewImage &image) {
            value["bufferView"] = image.bufferView;
            value["mimeType"] = image.mimeType;
        }
    } bi(value);
    boost::apply_visitor(bi, image);
}

void build(Json::Value &value, const TextureInfo &textureInfo)
{
    value = Json::objectValue;
    build(value["index"], textureInfo.index);
    build(value, "texCoord", textureInfo.texCoord);
    build(value, "scale", textureInfo.scale);
}

void build(Json::Value &value
           , const PbrMetallicRoughness &pbrMetallicRoughness)
{
    value = Json::objectValue;
    build(value, "baseColorTexture", pbrMetallicRoughness.baseColorTexture);
    build(value, "metallicFactor", pbrMetallicRoughness.metallicFactor);
    build(value, "roughnessFactor", pbrMetallicRoughness.roughnessFactor);
}

void build(Json::Value &value, const Material &material)
{
    value = Json::objectValue;
    build(value, "name", material.name);
    build(value, "pbrMetallicRoughness", material.pbrMetallicRoughness);

    if (material.extension_unlit) {
        auto &ext(extensions(value));
        ext["KHR_materials_unlit"] = Json::objectValue;
    }
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

void build(Json::Value &value, const GLTF &gltf)
{
    value = Json::objectValue;

    build(value["asset"], gltf.asset);
    build(value, "scenes", gltf.scenes);
    build(value, "scene", gltf.scene);
    build(value, "nodes", gltf.nodes);
    build(value, "meshes", gltf.meshes);
    build(value, "samplers", gltf.samplers);
    build(value, "images", gltf.images);
    build(value, "textures", gltf.textures);
    build(value, "materials", gltf.materials);
    build(value, "buffers", gltf.buffers);
    build(value, "bufferViews", gltf.bufferViews);
    build(value, "accessors", gltf.accessors);

    // extensions; TODO: dynamic?
    build(value, "extensionsUsed", gltf.extensionsUsed);
    build(value, "extensionsRequired", gltf.extensionsRequired);
}

} // namespace detail

void write(std::ostream &os, const GLTF &gltf)
{
    Json::Value value;
    detail::build(value, gltf);
    os.precision(15);
    Json::write(os, value);
}

void write(const boost::filesystem::path &path, const GLTF &gltf)
{
    LOG(info1) << "Saving glTF to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    write(f, gltf);
    f.close();
}

} // namespace gltf
