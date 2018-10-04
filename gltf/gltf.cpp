#include "utility/base64.hpp"
#include "utility/streams.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "./gltf.hpp"

namespace gltf {

namespace detail {

template <typename T>
void build(Json::Value &object, const T &value)
{
    object = value;
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

template <typename T>
void build(Json::Value &object, const char *name
         , const boost::optional<T> &value)
{
    if (value) { build(object[name], *value); }
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

void build(Json::Value &value, const Mesh &mesh)
{
    value = Json::objectValue;
    (void) mesh;
}

void build(Json::Value &value, const Texture &texture)
{
    value = Json::objectValue;
    value["sampler"] = texture.sampler;
    value["source"] = texture.source;
}


void build(Json::Value &value, const Image &image)
{
    struct BuildImage : public boost::static_visitor<void> {
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

        Json::Value &value;
    } bi(value);
    boost::apply_visitor(bi, image);
}

template <typename T>
void build(Json::Value &value, const std::vector<T> &list)
{
    value = Json::arrayValue;
    for (const auto &element : list) {
        build(value.append({}), element);
    }
}

void build(Json::Value &value, const Assets &assets)
{
    value = Json::objectValue;

    build(value["scenes"], assets.scenes);
    build(value, "scene", assets.scene);
    build(value["nodes"], assets.nodes);
    build(value["meshes"], assets.meshes);
    build(value["textures"], assets.textures);
    build(value["images"], assets.images);
}

} // namespace detail

void write(std::ostream &os, const Assets &assets)
{
    Json::Value value;
    detail::build(value, assets);
    os.precision(15);
    Json::write(os, value);
}

void write(const boost::filesystem::path &path, const Assets &assets)
{
    LOG(info1) << "Saving glTF assets to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    write(f, assets);
    f.close();
}

} // namespace gltf
