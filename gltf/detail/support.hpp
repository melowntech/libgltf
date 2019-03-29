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

#ifndef gltf_detail_support_hpp_included_
#define gltf_detail_support_hpp_included_

#include "jsoncpp/json.hpp"

#include "../gltf.hpp"

namespace gltf { namespace detail {

void parse(std::vector<std::string> &list, const Json::Value &value);
void parse(Indices &indices, const Json::Value &value);

void common(CommonBase &cb, const Json::Value &value);
void namedCommon(NamedCommonBase &ncb, const Json::Value &value);

void parse(Version &version, const Json::Value &value);

void parse(math::Point3d &point, const Json::Value &value);
void parse(math::Point4d &point, const Json::Value &value);
void parse(math::Matrix4 &matrix, const Json::Value &value);

void parse(ComponentValue &cv, const Json::Value &value);
void parse(ComponentType &ct, const Json::Value &value);
void parse(WrappingMode &wm, const Json::Value &value);
void parse(Target &target, const Json::Value &value);
void parse(PrimitiveMode &mode, const Json::Value &value);

} } // namespace gltf::detail

#endif // gltf_detail_support_hpp_included_
