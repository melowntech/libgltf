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

#ifndef gltf_meshloader_hpp_included_
#define gltf_meshloader_hpp_included_

#include "gltf.hpp"

namespace gltf {

class MeshLoader {
public:
    using Face = math::Point3_<unsigned int>;
    using Faces = std::vector<Face>;

    virtual ~MeshLoader();

    /** New mesh has been encountered.
     */
    virtual void mesh() = 0;

    /** Mesh vertices.
     */
    virtual void vertices(math::Points3d&&) = 0;

    /** Mesh texture coordinates.
     */
    virtual void tc(math::Points2d&&) = 0;

    /** Mexh faces. Indices are valid for both 3D and 2D vertices (i.e. vertices
     *  and texture coordinates.
     */
    virtual void faces(Faces &&faces) = 0;

    /** Image data.
     */
    virtual void image(const DataView &imageData) = 0;
};

/** Decode mesh from all scene nodes.
 *
 *  Each mesh is notified as separate nextMesh() call. Node structure is thus
 *  flattened.
 *
 * \param loaded mesh loader
 * \param model glTF model
 * \param scene scene to use, default to default scene
 *
 * Fails if there is no scene to load from.
 */
void decodeMesh(MeshLoader &loader, const Model &model
                , const math::Matrix4 &trafo
                = boost::numeric::ublas::identity_matrix<double>(4, 4)
                , const OptIndex &scene = boost::none);

} // namespace gltf

#endif // gltf_meshloader_hpp_included_
