#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "utility/streams.hpp"
#include "utility/format.hpp"
#include "utility/binaryio.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "./gltf.hpp"

namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace bin = utility::binaryio;

namespace gltf {

namespace detail {

struct Header {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t length;

    Header() : magic(0x46546C67), version(2), length() {}
};

void write(std::ostream &os, const Header &header)
{
    bin::write(os, header.magic);
    bin::write(os, header.version);
    bin::write(os, header.length);
}

std::string detectMimeType(const fs::path &filename) {
    const auto ext(filename.extension());
    if (ext == ".bin") { return "application/octet-stream"; }
    if (ext == ".jpg") { return "image/jpeg"; }
    if (ext == ".png") { return "image/png"; }
    return "application/octet-stream";
}

struct ExternalFile {
    fs::path path;
    std::string mimeType;

    ExternalFile(const fs::path &path)
        : path(path), mimeType(detectMimeType(path))
    {}

    typedef std::vector<ExternalFile> list;
};

bool localPath(const std::string &uri)
{
    if (ba::starts_with(uri, "http:")) { return false; }
    if (ba::starts_with(uri, "https:")) { return false; }
    if (ba::starts_with(uri, "file:")) { return false; }

    return true;
}

bool localPath(const OptString &uri)
{
    if (!uri) { return false; }
    return localPath(*uri);
}

ExternalFile::list collectFiles(GLTF gltf, const fs::path &srcDir)
{
    ExternalFile::list files;

    for (const auto &buffer : gltf.buffers) {
        if (!localPath(buffer.uri)) { continue; }
        files.emplace_back(srcDir / *buffer.uri);
    }

    for (const auto &image : gltf.images) {
        const auto *rimage(boost::get<ReferencedImage>(&image));
        if (!rimage) { continue; }

        if (!localPath(rimage->uri)) { continue; }
        files.emplace_back(srcDir / rimage->uri);
    }

    return files;
}

} // namespace detail

void glb(const fs::path &path, const GLTF &gltf, const fs::path &srcDir)
{
    LOG(info1) << "Generating GLB in " << path  << ".";

    utility::ofstreambuf os(path.string());

    const auto files(detail::collectFiles(gltf, srcDir));

    for (const auto &file : files) {
        LOG(info4) << "file: " << file.path << ": " << file.mimeType;
    }

    // NB: placeholder for header, will be rewritten when total size is known.
    detail::Header header;
    write(os, header);

    (void) gltf;
    (void) srcDir;
}

} // namespace gltf
