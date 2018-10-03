#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "./gltf.hpp"

namespace gltf {

namespace detail {


void build(Json::Value &value, const char *name, const Indices &indices)
{
    if (indices.empty()) { return; }
    auto &list(value[name] = Json::arrayValue);
    for (auto index : indices) { list.append(index); }
}

void build(Json::Value &value, const Assets &assets)
{
    value = Json::objectValue;

    auto &scenes(value["scenes"] = Json::arrayValue);

    for (const auto &scene : assets.scenes) {
        auto &jScene(scenes.append(Json::objectValue));
        if (scene.name) { jScene["name"] = *scene.name; }
        build(jScene, "nodes", scene.nodes);
    }

    if (assets.scene) { value["scene"] = *assets.scene; }
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
