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

#include "dbglog/dbglog.hpp"

#include "gltf.hpp"

namespace gltf {

namespace {

void validateFloat3(const Accessor &a, AttributeSemantic semantic) {
    if ((a.type != AttributeType::vec3)
        || (a.componentType != ComponentType::float_)
        || a.normalized)
    {
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be VEC3(float).";
    }
}

void validateFloat4(const Accessor &a, AttributeSemantic semantic) {
    if ((a.type != AttributeType::vec4)
        || (a.componentType != ComponentType::float_)
        || a.normalized)
    {
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be VEC4(float).";
    }
}

void validateTexCoord(const Accessor &a, AttributeSemantic semantic) {
    if (a.type != AttributeType::vec2) {
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be VEC2(float|normalized unsinged byte"
            "|normalized unsinged short).";
    }

    switch (a.componentType) {
    case ComponentType::float_:
        if (a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be VEC2(float).";
        }
        return;

    case ComponentType::ubyte:
        if (!a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be VEC2(normalized unsigned byte).";
        }
        return;

    case ComponentType::ushort:
        if (!a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be VEC2(normalized unsigned short).";
        }
        return;

    default:
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be VEC2(float|normalized unsigned byte"
            "|normalized unsigned short).";
        return;
    }
}

void validateColor(const Accessor &a, AttributeSemantic semantic) {
    if ((a.type != AttributeType::vec3)
        && (a.type != AttributeType::vec4))
    {
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be (VEC3|VEC4)(float|normalized unsinged byte"
            "|normalized unsinged short).";
    }

    switch (a.componentType) {
    case ComponentType::float_:
        if (a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be " << a.type << "(float).";
        }
        return;

    case ComponentType::ubyte:
        if (!a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be " << a.type << "(normalized unsigned byte).";
        }
        return;

    case ComponentType::ushort:
        if (!a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be " << a.type << "(normalized unsigned short).";
        }
        return;

    default:
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be " << a.type << "(float|normalized unsinged byte"
            "|normalized unsinged short).";
        return;
    }
}

void validateJoints(const Accessor &a, AttributeSemantic semantic) {
    if (a.type != AttributeType::vec4) {
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be VEC4(unsigned byte|unsinged short).";
    }

    switch (a.componentType) {
    case ComponentType::ubyte:
        if (a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be VEC4(unsigned byte).";
        }
        return;

    case ComponentType::ushort:
        if (a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be VEC4(unsigned short).";
        }
        return;

    default:
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be VEC4(unsigned byte|unsigned short).";
        return;
    }
}

void validateWeights(const Accessor &a, AttributeSemantic semantic) {
    if (a.type != AttributeType::vec3) {
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be VEC4float|normalized unsinged byte"
            "|normalized unsinged short).";
    }

    switch (a.componentType) {
    case ComponentType::float_:;
        if (a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be VEC4(float).";
        }
        return;

    case ComponentType::ubyte:
        if (!a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be VEC4(normalized unsigned byte).";
        }
        return;

    case ComponentType::ushort:
        if (!a.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Attribute with semantic <" << semantic
                << "> must be VEC4(normalized unsigned short).";
        }
        return;

    default:
        LOGTHROW(err1, std::runtime_error)
            << "Attribute with semantic <" << semantic
            << "> must be VEC4(float|normalized unsinged byte"
            "|normalized unsinged short).";
        return;
    }
}

} // namespace

void Accessor::validate(AttributeSemantic semantic) const
{
    switch (semantic) {
    case AttributeSemantic::position:
    case AttributeSemantic::normal:
        return validateFloat3(*this, semantic);

    case AttributeSemantic::tangent:
        return validateFloat4(*this, semantic);

    case AttributeSemantic::texCoord0:
    case AttributeSemantic::texCoord1:
        return validateTexCoord(*this, semantic);

    case AttributeSemantic::color0:
        return validateColor(*this, semantic);

    case AttributeSemantic::joints0:
        return validateJoints(*this, semantic);

    case AttributeSemantic::weights0:
        return validateWeights(*this, semantic);
    }
}

} // namespace gltf
