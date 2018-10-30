#include <map>
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
    std::uint32_t version;
    std::uint32_t length;

    GlbHeader() : version(2), length() {}

    static std::size_t size() { return 12; }
};

struct ChunkHeader {
    enum class Type : std::uint32_t {
        json = 0x4e4f534a
        , bin = 0x004e4942
    };

    std::uint32_t length;
    Type type;

    ChunkHeader(Type type) : length(), type(type) {}

    static std::size_t size() { return 8; }
};

void write(std::ostream &os, const GlbHeader &header)
{
    os << "glTF";
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

using FileData = boost::variant<fs::path, const Data*>;

struct ExternalFile {
    FileData data;
    std::string mimeType;
    std::size_t byteLength;
    std::size_t offset;
    std::size_t padding;
    std::size_t size;

    ExternalFile(const fs::path &path, std::size_t offset)
        : data(path), mimeType(detectMimeType(path))
        , byteLength(utility::fileSize(path))
        , offset(offset)
        , padding(computePadding(byteLength))
        , size(byteLength + padding)
    {}

    ExternalFile(const Data &srcData, std::size_t offset)
        : data(&srcData), byteLength(srcData.size())
        , offset(offset)
        , padding(computePadding(byteLength))
        , size(byteLength + padding)
    {}

    typedef std::vector<ExternalFile> list;
};

struct ExternalFiles {
    ExternalFile::list files;
    Buffers buffers;
    BufferView::list bufferViews;
    Images images;
    std::size_t dataLength;

    ExternalFiles() : dataLength() {}
};

using BufferMappingValue = std::pair<Index, bool>;
using BufferMapping = std::vector<BufferMappingValue>;

struct BufferAdder : public boost::static_visitor<void> {
    const fs::path &srcDir;
    BufferMapping &bufferMapping;
    ExternalFiles &ef;
    std::size_t &offset;

    BufferAdder(const fs::path &srcDir, BufferMapping &bufferMapping
                , ExternalFiles &ef, std::size_t &offset)
        : srcDir(srcDir), bufferMapping(bufferMapping), ef(ef), offset(offset)
    {}

    void operator()(const InlineBuffer &buffer) {
        // inline, embed
        bufferMapping.emplace_back(ef.files.size(), false);
        ef.files.emplace_back(buffer.data, offset);
        offset += ef.files.back().size;
    }

    void operator()(const ExternalBuffer &buffer) {
        if (localPath(buffer.uri)) {
            // local, embed
            // TODO: embed file with the same uri only once
            bufferMapping.emplace_back(ef.files.size(), false);
            ef.files.emplace_back(srcDir / *buffer.uri, offset);
            offset += ef.files.back().size;
        } else {
            // external
            bufferMapping.emplace_back(ef.buffers.size(), true);
            ef.buffers.push_back(buffer);
        }
    }
};

struct ImageAdder : public boost::static_visitor<void> {
    const fs::path &srcDir;
    ExternalFiles &ef;
    std::size_t &offset;

    ImageAdder(const fs::path &srcDir, ExternalFiles &ef, std::size_t &offset)
        : srcDir(srcDir), ef(ef), offset(offset)
    {}

    void operator()(const ExternalImage &image) {
        if (localPath(image.uri)) {
            // local, embed
            // TODO: embed image with the same uri only once

            ef.files.emplace_back(srcDir / image.uri, offset);
            const auto &file(ef.files.back());
            offset += file.size;

            BufferView bv(0, file.byteLength);
            bv.byteOffset = file.offset;

            BufferViewImage bvi(ef.bufferViews.size());
            bvi.mimeType = file.mimeType;

            ef.bufferViews.push_back(bv);
            ef.images.push_back(bvi);
        } else {
            // keep
            ef.images.push_back(image);
        }
    }

    void operator()(const BufferViewImage &image) {
        ef.images.push_back(image);
    }

    void operator()(const InlineImage &image) {
        // embed local image
        ef.files.emplace_back(image.data, offset);
        const auto &file(ef.files.back());
        offset += file.size;

        BufferView bv(0, file.byteLength);
        bv.byteOffset = file.offset;

        BufferViewImage bvi(ef.bufferViews.size());
        bvi.mimeType = image.mimeType;

        ef.bufferViews.push_back(bv);
        ef.images.push_back(bvi);
    }
};

ExternalFiles collectFiles(const GLTF &gltf, const fs::path &srcDir)
{
    ExternalFiles ef;

    ef.buffers.push_back(ExternalBuffer());
    auto &embedded(boost::get<ExternalBuffer>(ef.buffers.back()));

    auto &offset(embedded.byteLength);

    BufferMapping bufferMapping;

    // buffers
    {
        BufferAdder adder(srcDir, bufferMapping, ef, offset);
        for (Index bufferId(0), ebufferId(gltf.buffers.size());
             bufferId != ebufferId; ++bufferId)
        {
            boost::apply_visitor(adder, gltf.buffers[bufferId]);
        }
    }

    // generate new buffer views for updated buffers
    for (Index bvId(0), ebvId(gltf.bufferViews.size());
         bvId != ebvId; ++bvId)
    {
        auto bv(gltf.bufferViews[bvId]);
        const auto &mapping(bufferMapping[bv.buffer]);
        if (mapping.second) {
            // external, just rewrite
            bv.buffer = mapping.first;
        } else {
            // internal
            // index points to file in ef.files
            bv.buffer = 0; // map to the first (embedded) buffer

            const auto &file(ef.files[mapping.first]);

            if (bv.byteOffset) {
                *bv.byteOffset += file.offset;
            } else {
                bv.byteOffset = file.offset;
            }
        }
        ef.bufferViews.push_back(bv);
    }

    // images
    {
        ImageAdder adder(srcDir, ef, offset);
        for (const auto &image : gltf.images) {
            boost::apply_visitor(adder, image);
        }
    }

    // pad whole embedded buffer
    offset += computePadding(offset);

    ef.dataLength = offset;

    return ef;
}

void write(std::ostream &os, const ExternalFiles &ef)
{
    struct FileWriter : public boost::static_visitor<void> {
        FileWriter(std::ostream &os) : os(os) {}
        std::ostream &os;

        void operator()(const fs::path &path) {
            utility::ifstreambuf is(path.string());
            os << is.rdbuf();
            is.close();
        }

        void operator()(const Data *data) {
            bin::write(os, data->data(), data->size());
        }
    };

    FileWriter writer(os);
    for (const auto &file : ef.files) {
        boost::apply_visitor(writer, file.data);

        // pad
        for (auto padding(file.padding); padding; --padding) {
            os << '\0';
        }
    }
}

} // namespace detail

void glb(std::ostream &os, const GLTF &srcGltf, const fs::path &srcDir)
{
    auto ef(detail::collectFiles(srcGltf, srcDir));

    auto gltf(srcGltf);
    std::swap(gltf.buffers, ef.buffers);
    std::swap(gltf.bufferViews, ef.bufferViews);
    std::swap(gltf.images, ef.images);

    const auto json([&]() -> std::string
    {
        // serialize glTF JSON and pad it with spaces
        std::ostringstream tmp;
        write(tmp, gltf);
        auto padding(detail::computePadding(tmp.tellp()));
        while (padding--) { tmp << ' '; }
        return tmp.str();
    }());

    std::size_t fileSize
        = detail::GlbHeader::size()
        + detail::ChunkHeader::size() + json.size()
        + detail::ChunkHeader::size() + ef.dataLength;


    // sanity check
    if (fileSize > std::numeric_limits<std::uint32_t>::max()) {
        LOGTHROW(err2, std::runtime_error)
            << "Data too large to be represented in GLB. "
            "Use directory based storage instead.";
    }

    // write header
    {
        detail::GlbHeader header;
        header.length = fileSize;
        detail::write(os, header);
    }

    // write glTF JSON chunk
    {
        detail::ChunkHeader header(detail::ChunkHeader::Type::json);
        header.length = json.size();
        detail::write(os, header);
        bin::write(os, json.data(), json.size());
    }

    // write BIN chunk
    {
        detail::ChunkHeader header(detail::ChunkHeader::Type::bin);
        header.length = ef.dataLength;
        detail::write(os, header);
        detail::write(os, ef);
    }
}

void glb(const fs::path &path, const GLTF &srcGltf, const fs::path &srcDir)
{
    LOG(info1) << "Generating GLB in " << path  << ".";
    utility::ofstreambuf os(path.string());
    glb(os, srcGltf, srcDir);
    os.close();
}

} // namespace gltf
