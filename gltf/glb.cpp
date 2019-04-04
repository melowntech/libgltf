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

#include <map>
#include <limits>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/format.hpp"
#include "utility/binaryio.hpp"
#include "utility/filesystem.hpp"
#include "utility/enum-io.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "gltf.hpp"
#include "detail.hpp"

namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace bio = boost::iostreams;

namespace bin = utility::binaryio;

namespace gltf {

namespace detail {

const char MAGIC[4] = { 'g', 'l', 'T', 'F' };

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
       , json_v1 = 0x0 // v1; deprecated
    };

    std::uint32_t length;
    Type type;

    ChunkHeader(std::uint32_t length = 0, Type type = Type())
        : length(length), type(type)
    {}

    static std::size_t size() { return 8; }
};

UTILITY_GENERATE_ENUM_IO(ChunkHeader::Type,
                         ((json))((bin))((json_v1)))

void write(std::ostream &os, const GlbHeader &header)
{
    bin::write(os, MAGIC);
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

ExternalFiles collectFiles(const Model &model, const fs::path &srcDir)
{
    ExternalFiles ef;

    ef.buffers.push_back(ExternalBuffer());
    auto &embedded(boost::get<ExternalBuffer>(ef.buffers.back()));

    auto &offset(embedded.byteLength);

    BufferMapping bufferMapping;

    // buffers
    {
        BufferAdder adder(srcDir, bufferMapping, ef, offset);
        for (Index bufferId(0), ebufferId(model.buffers.size());
             bufferId != ebufferId; ++bufferId)
        {
            boost::apply_visitor(adder, model.buffers[bufferId]);
        }
    }

    // generate new buffer views for updated buffers
    for (Index bvId(0), ebvId(model.bufferViews.size());
         bvId != ebvId; ++bvId)
    {
        auto bv(model.bufferViews[bvId]);
        const auto &mapping(bufferMapping[bv.buffer]);
        if (mapping.second) {
            // external, just rewrite
            bv.buffer = mapping.first;
        } else {
            // internal
            // index points to file in ef.files
            bv.buffer = 0; // map to the first (embedded) buffer

            const auto &file(ef.files[mapping.first]);

            bv.byteOffset += file.offset;
        }
        ef.bufferViews.push_back(bv);
    }

    // images
    {
        ImageAdder adder(srcDir, ef, offset);
        for (const auto &image : model.images) {
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

void glb(std::ostream &os, const Model &srcModel, const fs::path &srcDir)
{
    auto ef(detail::collectFiles(srcModel, srcDir));

    auto model(srcModel);
    std::swap(model.buffers, ef.buffers);
    std::swap(model.bufferViews, ef.bufferViews);
    std::swap(model.images, ef.images);

    const auto json([&]() -> std::string
    {
        // serialize model JSON and pad it with spaces
        std::ostringstream tmp;
        write(tmp, model);
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

    // write model JSON chunk
    {
        detail::ChunkHeader header
            (json.size(), detail::ChunkHeader::Type::json);
        detail::write(os, header);
        bin::write(os, json.data(), json.size());
    }

    // write BIN chunk
    {
        detail::ChunkHeader header
            (ef.dataLength, detail::ChunkHeader::Type::bin);
        detail::write(os, header);
        detail::write(os, ef);
    }
}

namespace detail {

void read(std::istream &is, GlbHeader &header
          , const boost::filesystem::path &path, bool magicRead)
{
    if (!magicRead) {
        // magic reading and checking is skipped if the caller claims it has
        // been already read
        char magic[4];
        bin::read(is, magic);
        if (std::memcmp(magic, MAGIC, sizeof(MAGIC))) {
            LOGTHROW(err2, std::runtime_error)
                << "File " << path << " is not a glTF Model file.";
        }
    }

    bin::read(is, header.version);
    bin::read(is, header.length);
}

void read(std::istream &is, ChunkHeader &header, const fs::path &path
          , ChunkHeader::Type expectedType, int index)
{
    bin::read(is, header.length);
    header.type = static_cast<ChunkHeader::Type>
        (bin::read<std::uint32_t>(is));

    if (expectedType != header.type) {
        LOGTHROW(err2, std::runtime_error)
            << "GLB file " << path << ": chunk #" << index
            << " has not " << expectedType << " type.";
    }
}

std::uint32_t readJson(Json::Value &json, std::istream &is
                       , const boost::filesystem::path &path
                       , ChunkHeader::Type type = ChunkHeader::Type::json)
{
    ChunkHeader header;
    read(is, header, path, type, 0);

    // read feature table to temporary buffer
    std::vector<char> buf(header.length);
    bin::read(is, buf);

    // wrap buffer to a stream
    bio::stream_buffer<bio::array_source>
        buffer(buf.data(), buf.data() + buf.size());
    std::istream s(&buffer);
    s.exceptions(std::ios::badbit | std::ios::failbit);

    // and read
    if (!read(s, json)) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to read JSON chunk from file "
            << path << ".";
    }

    return header.length;
}

Data readData(std::istream &is, const boost::filesystem::path &path
              , const boost::optional<ChunkHeader> &ch = boost::none)
{
    ChunkHeader header;
    if (ch) {
        header = *ch;
    } else {
        read(is, header, path, ChunkHeader::Type::bin, 1);
    }

    Data data(header.length);
    bin::read(is, data);
    return data;
}

} // namespace detail

void glb(const fs::path &path, const Model &srcModel, const fs::path &srcDir)
{
    LOG(info1) << "Generating GLB in " << path  << ".";
    utility::ofstreambuf os(path.string());
    glb(os, srcModel, srcDir);
    os.close();
}

Model glb(std::istream &is, const fs::path &path, bool magicRead)
{

    detail::GlbHeader header;
    detail::read(is, header, path, magicRead);

    Model model;
    Json::Value content;
    InlineBuffer buffer;

    switch (header.version) {
    case 1: {
        // only json chunk has header
        const auto jsonSize
            (detail::readJson
             (content, is, path, detail::ChunkHeader::Type::json_v1));
        const auto soFar(header.size() + detail::ChunkHeader::size()
                         + jsonSize);
        if (soFar < header.length) {
            detail::ChunkHeader ch(header.length - soFar
                                   , detail::ChunkHeader::Type::bin);
            buffer.set(detail::readData(is, path, ch));
        } else if (soFar > header.length) {
            LOGTHROW(err2, std::runtime_error)
                << "Invalid size of binary v. 1.0 chunk.";
        }
        // FIXME: v 1.0 parsing not implemented so far
    } break;

    case 2:
        detail::readJson(content, is, path);
        buffer.set(detail::readData(is, path));
        break;

    default:
        LOGTHROW(err2, std::runtime_error)
            << "Unsupported GLB version " << header.version << ".";
        break;
    }

    read(model, content, path, header.version);

    // prepend bin into buffers
    model.buffers.insert(model.buffers.begin(), std::move(buffer));

    return model;
}

} // namespace gltf
