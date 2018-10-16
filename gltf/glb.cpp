#include <limits>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/format.hpp"
#include "utility/binaryio.hpp"
#include "utility/filesystem.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "./gltf.hpp"

namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace bin = utility::binaryio;

namespace gltf {

namespace detail {

struct GlbHeader {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t length;

    GlbHeader() : magic(0x46546c67), version(2), length() {}
};

struct ChunkHeader {
    enum class Type : std::uint32_t {
        json = 0x4e4f534a
        , bin = 0x004e4942
    };

    std::uint32_t length;
    Type type;
};

void write(std::ostream &os, const GlbHeader &header)
{
    bin::write(os, header.magic);
    bin::write(os, header.version);
    bin::write(os, header.length);
}

void write(std::ostream &os, const ChunkHeader &header)
{
    bin::write(os, header.length);
    bin::write(os, static_cast<std::uint32_t>(header.type));
}

inline std::string detectMimeType(const fs::path &filename) {
    const auto ext(filename.extension());
    if (ext == ".bin") { return "application/octet-stream"; }
    if (ext == ".jpg") { return "image/jpeg"; }
    if (ext == ".png") { return "image/png"; }
    return "application/octet-stream";
}

inline std::size_t computePadding(std::size_t size)
{
    const auto rem(size & 0x03);
    if (!rem) { return 0; }
    return 4 - rem;
}

inline bool localPath(const std::string &uri)
{
    if (ba::starts_with(uri, "http:")) { return false; }
    if (ba::starts_with(uri, "https:")) { return false; }
    if (ba::starts_with(uri, "file:")) { return false; }

    return true;
}

inline bool localPath(const OptString &uri)
{
    if (!uri) { return false; }
    return localPath(*uri);
}

struct ExternalFile {
    fs::path path;
    std::string mimeType;
    BufferView bufferView;
    std::size_t size;

    ExternalFile(const fs::path &path, std::size_t offset
                 , Index bufferId = -1)
        : path(path), mimeType(detectMimeType(path))
        , bufferView(bufferId, utility::fileSize(path))
        , size(bufferView.byteLength + computePadding(bufferView.byteLength))
    {
        bufferView.byteOffset = offset;
    }

    typedef std::vector<ExternalFile> list;
};

struct ExternalFiles {
    ExternalFile::list files;
    std::size_t size;

    ExternalFiles() : size() {}
};

ExternalFiles collectFiles(GLTF gltf, const fs::path &srcDir)
{
    ExternalFiles ef;

    std::size_t offset(0);

    for (Index bufferId(0), ebufferId(gltf.buffers.size());
         bufferId != ebufferId; ++bufferId)
    {
        const auto &buffer(gltf.buffers[bufferId]);
        if (!localPath(buffer.uri)) { continue; }
        ef.files.emplace_back(srcDir / *buffer.uri, offset, bufferId);
        offset += ef.files.back().size;
    }

    for (const auto &image : gltf.images) {
        const auto *rimage(boost::get<ReferencedImage>(&image));
        if (!rimage) { continue; }

        if (!localPath(rimage->uri)) { continue; }
        ef.files.emplace_back(srcDir / rimage->uri, offset);
        offset += ef.files.back().size;
    }

    ef.size = offset + computePadding(offset);

    return ef;
}

} // namespace detail

void glb(const fs::path &path, const GLTF &gltf, const fs::path &srcDir)
{
    LOG(info1) << "Generating GLB in " << path  << ".";

    utility::ofstreambuf os(path.string());

    const auto ef(detail::collectFiles(gltf, srcDir));

    for (const auto &file : ef.files) {
        LOG(info4) << "file: " << file.path << ": " << file.mimeType
                   << ", " << file.bufferView.byteLength
                   << ", " << file.bufferView.buffer;
    }
    LOG(info4) << "chunk size: " << ef.size;
    if (ef.size > std::numeric_limits<std::uint32_t>::max()) {
        LOGTHROW(err2, std::runtime_error)
            << "Data too large to be represented in GLB. "
            "Use directory based storage instead.";
    }

    // NB: placeholder for header, will be rewritten when total size is known.
    detail::GlbHeader header;
    write(os, header);

    (void) gltf;
    (void) srcDir;
}

} // namespace gltf
