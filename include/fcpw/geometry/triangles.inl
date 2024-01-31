namespace fcpw {

template<size_t DIM>
inline Triangle<DIM>::Triangle()
{
    indices[0] = -1;
    indices[1] = -1;
    indices[2] = -1;
    soup = nullptr;
    pIndex = -1;
}

template<size_t DIM>
inline BoundingBox<DIM> Triangle<DIM>::boundingBox() const
{
    const Vector<DIM>& pa = soup->positions[indices[0]];
    const Vector<DIM>& pb = soup->positions[indices[1]];
    const Vector<DIM>& pc = soup->positions[indices[2]];

    BoundingBox<DIM> box(pa);
    box.expandToInclude(pb);
    box.expandToInclude(pc);

    return box;
}

template<size_t DIM>
inline Vector<DIM> Triangle<DIM>::centroid() const
{
    const Vector<DIM>& pa = soup->positions[indices[0]];
    const Vector<DIM>& pb = soup->positions[indices[1]];
    const Vector<DIM>& pc = soup->positions[indices[2]];

    return (pa + pb + pc)/3.0f;
}

template<size_t DIM>
inline float Triangle<DIM>::surfaceArea() const
{
    return 0.5f*normal().norm();
}

template<size_t DIM>
inline float Triangle<DIM>::signedVolume() const
{
    const Vector<DIM>& pa = soup->positions[indices[0]];
    const Vector<DIM>& pb = soup->positions[indices[1]];
    const Vector<DIM>& pc = soup->positions[indices[2]];

    return pa.cross(pb).dot(pc)/6.0f;
}

template<size_t DIM>
inline Vector<DIM> Triangle<DIM>::normal(bool normalize) const
{
    const Vector<DIM>& pa = soup->positions[indices[0]];
    const Vector<DIM>& pb = soup->positions[indices[1]];
    const Vector<DIM>& pc = soup->positions[indices[2]];

    Vector<DIM> v1 = pb - pa;
    Vector<DIM> v2 = pc - pa;

    Vector<DIM> n = v1.cross(v2);
    return normalize ? n.normalized() : n;
}

template<size_t DIM>
inline Vector<DIM> Triangle<DIM>::normal(int vIndex, int eIndex) const
{
    if (soup->vNormals.size() > 0 && vIndex >= 0) {
        return soup->vNormals[indices[vIndex]];
    }

    if (soup->eNormals.size() > 0 && eIndex >= 0) {
        return soup->eNormals[soup->eIndices[3*pIndex + eIndex]];
    }

    return normal(true);
}

template<size_t DIM>
inline Vector<DIM> Triangle<DIM>::normal(const Vector2& uv) const
{
    int vIndex = -1;
    if (uv[0] >= oneMinusEpsilon && uv[1] <= epsilon) vIndex = 0; // (1, 0, 0)
    else if (uv[0] <= epsilon && uv[1] >= oneMinusEpsilon) vIndex = 1; // (0, 1, 0)
    else if (uv[0] <= epsilon && uv[1] <= epsilon) vIndex = 2; // (0, 0, 1)

    int eIndex = -1;
    if (vIndex == -1) {
        if (uv[0] <= epsilon) eIndex = 1; // (0, 1 - w, w)
        else if (uv[1] <= epsilon) eIndex = 2; // (1 - w, 0, w)
        else if (uv[0] + uv[1] >= oneMinusEpsilon) eIndex = 0; // (1 - v, v, 0)
    }

    return normal(vIndex, eIndex);
}

template<size_t DIM>
inline Vector2 Triangle<DIM>::barycentricCoordinates(const Vector<DIM>& p) const
{
    const Vector<DIM>& pa = soup->positions[indices[0]];
    const Vector<DIM>& pb = soup->positions[indices[1]];
    const Vector<DIM>& pc = soup->positions[indices[2]];

    Vector<DIM> v1 = pb - pa;
    Vector<DIM> v2 = pc - pa;
    Vector<DIM> v3 = p - pa;

    float d11 = v1.dot(v1);
    float d12 = v1.dot(v2);
    float d22 = v2.dot(v2);
    float d31 = v3.dot(v1);
    float d32 = v3.dot(v2);
    float denom = d11*d22 - d12*d12;
    float v = (d22*d31 - d12*d32)/denom;
    float w = (d11*d32 - d12*d31)/denom;

    return Vector2(1.0f - v - w, v);
}

template<size_t DIM>
inline float Triangle<DIM>::samplePoint(float *randNums, Vector2& uv, Vector<DIM>& p, Vector<DIM>& n) const
{
    const Vector<DIM>& pa = soup->positions[indices[0]];
    const Vector<DIM>& pb = soup->positions[indices[1]];
    const Vector<DIM>& pc = soup->positions[indices[2]];

    n = (pb - pa).cross(pc - pa);
    float area = n.norm();
    float u1 = std::sqrt(randNums[1]);
    float u2 = randNums[2];
    float u = 1.0f - u1;
    float v = u2*u1;
    float w = 1.0f - u - v;
    uv = Vector2(u, v);
    p = pa*u + pb*v + pc*w;
    n /= area;

    return 2.0f/area;
}

template<size_t DIM>
inline Vector2 Triangle<DIM>::textureCoordinates(const Vector2& uv) const
{
    if (soup->tIndices.size() > 0) {
        const Vector2& pa = soup->textureCoordinates[soup->tIndices[3*pIndex]];
        const Vector2& pb = soup->textureCoordinates[soup->tIndices[3*pIndex + 1]];
        const Vector2& pc = soup->textureCoordinates[soup->tIndices[3*pIndex + 2]];

        float u = uv[0];
        float v = uv[1];
        float w = 1.0f - u - v;

        return pa*u + pb*v + pc*w;
    }

    return Vector2(-1, -1);
}

template<size_t DIM>
inline float Triangle<DIM>::angle(int vIndex) const
{
    const Vector<DIM>& pa = soup->positions[indices[vIndex]];
    const Vector<DIM>& pb = soup->positions[indices[(vIndex + 1)%3]];
    const Vector<DIM>& pc = soup->positions[indices[(vIndex + 2)%3]];

    Vector<DIM> u = (pb - pa).normalized();
    Vector<DIM> v = (pc - pa).normalized();

    return std::acos(std::max(-1.0f, std::min(1.0f, u.dot(v))));
}

template<size_t DIM>
inline void Triangle<DIM>::split(int dim, float splitCoord, BoundingBox<3>& boxLeft,
                            BoundingBox<3>& boxRight) const
{
    for (int i = 0; i < 3; i++) {
        const Vector<DIM>& pa = soup->positions[indices[i]];
        const Vector<DIM>& pb = soup->positions[indices[(i + 1)%3]];

        if (pa[dim] <= splitCoord && pb[dim] <= splitCoord) {
            const Vector<DIM>& pc = soup->positions[indices[(i + 2)%3]];

            if (pc[dim] <= splitCoord) {
                boxLeft = BoundingBox<3>(pa);
                boxLeft.expandToInclude(pb);
                boxLeft.expandToInclude(pc);
                boxRight = BoundingBox<3>();

            } else {
                Vector<DIM> u = pa - pc;
                Vector<DIM> v = pb - pc;
                float t = std::clamp((splitCoord - pc[dim])/u[dim], 0.0f, 1.0f);
                float s = std::clamp((splitCoord - pc[dim])/v[dim], 0.0f, 1.0f);

                boxLeft = BoundingBox<3>(pc + u*t);
                boxLeft.expandToInclude(pc + v*s);
                boxRight = boxLeft;
                boxLeft.expandToInclude(pa);
                boxLeft.expandToInclude(pb);
                boxRight.expandToInclude(pc);
            }

            break;

        } else if (pa[dim] >= splitCoord && pb[dim] >= splitCoord) {
            const Vector<DIM>& pc = soup->positions[indices[(i + 2)%3]];

            if (pc[dim] >= splitCoord) {
                boxRight = BoundingBox<3>(pa);
                boxRight.expandToInclude(pb);
                boxRight.expandToInclude(pc);
                boxLeft = BoundingBox<3>();

            } else {
                Vector<DIM> u = pa - pc;
                Vector<DIM> v = pb - pc;
                float t = std::clamp((splitCoord - pc[dim])/u[dim], 0.0f, 1.0f);
                float s = std::clamp((splitCoord - pc[dim])/v[dim], 0.0f, 1.0f);

                boxRight = BoundingBox<3>(pc + u*t);
                boxRight.expandToInclude(pc + v*s);
                boxLeft = boxRight;
                boxRight.expandToInclude(pa);
                boxRight.expandToInclude(pb);
                boxLeft.expandToInclude(pc);
            }

            break;
        }
    }
}

template<size_t DIM>
inline int Triangle<DIM>::intersect(Ray<3>& r, std::vector<Interaction<3>>& is,
                               bool checkForOcclusion, bool recordAllHits) const
{
    // Möller–Trumbore intersection algorithm
    const Vector<DIM>& pa = soup->positions[indices[0]];
    const Vector<DIM>& pb = soup->positions[indices[1]];
    const Vector<DIM>& pc = soup->positions[indices[2]];
    is.clear();

    Vector<DIM> v1 = pb - pa;
    Vector<DIM> v2 = pc - pa;
    Vector<DIM> p = r.d.cross(v2);
    float det = v1.dot(p);

    // ray and triangle are parallel if det is close to 0
    if (std::fabs(det) <= epsilon) return 0;
    float invDet = 1.0f/det;

    Vector<DIM> s = r.o - pa;
    float v = s.dot(p)*invDet;
    if (v < 0 || v > 1) return 0;

    Vector<DIM> q = s.cross(v1);
    float w = r.d.dot(q)*invDet;
    if (w < 0 || v + w > 1) return 0;

    float t = v2.dot(q)*invDet;
    if (t >= 0.0f && t <= r.tMax) {
        if (checkForOcclusion) return 1;
        auto it = is.emplace(is.end(), Interaction<3>());
        it->d = t;
        it->p = pa + v1*v + v2*w;
        it->n = v1.cross(v2).normalized();
        it->uv[0] = 1.0f - v - w;
        it->uv[1] = v;
        it->primitiveIndex = pIndex;

        return 1;
    }

    return 0;
}

template<size_t DIM>
inline float findClosestPointTriangle(const Vector<DIM>& pa, const Vector<DIM>& pb, const Vector<DIM>& pc,
                                      const Vector<DIM>& x, Vector<DIM>& pt, Vector2& t)
{
    // source: real time collision detection
    // check if x in vertex region outside pa
    Vector<DIM> ab = pb - pa;
    Vector<DIM> ac = pc - pa;
    Vector<DIM> ax = x - pa;
    float d1 = ab.dot(ax);
    float d2 = ac.dot(ax);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        // barycentric coordinates (1, 0, 0)
        t[0] = 1.0f;
        t[1] = 0.0f;
        pt = pa;
        return (x - pt).norm();
    }

    // check if x in vertex region outside pb
    Vector<DIM> bx = x - pb;
    float d3 = ab.dot(bx);
    float d4 = ac.dot(bx);
    if (d3 >= 0.0f && d4 <= d3) {
        // barycentric coordinates (0, 1, 0)
        t[0] = 0.0f;
        t[1] = 1.0f;
        pt = pb;
        return (x - pt).norm();
    }

    // check if x in vertex region outside pc
    Vector<DIM> cx = x - pc;
    float d5 = ab.dot(cx);
    float d6 = ac.dot(cx);
    if (d6 >= 0.0f && d5 <= d6) {
        // barycentric coordinates (0, 0, 1)
        t[0] = 0.0f;
        t[1] = 0.0f;
        pt = pc;
        return (x - pt).norm();
    }

    // check if x in edge region of ab, if so return projection of x onto ab
    float vc = d1*d4 - d3*d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        // barycentric coordinates (1 - v, v, 0)
        float v = d1/(d1 - d3);
        t[0] = 1.0f - v;
        t[1] = v;
        pt = pa + ab*v;
        return (x - pt).norm();
    }

    // check if x in edge region of ac, if so return projection of x onto ac
    float vb = d5*d2 - d1*d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        // barycentric coordinates (1 - w, 0, w)
        float w = d2/(d2 - d6);
        t[0] = 1.0f - w;
        t[1] = 0.0f;
        pt = pa + ac*w;
        return (x - pt).norm();
    }

    // check if x in edge region of bc, if so return projection of x onto bc
    float va = d3*d6 - d5*d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        // barycentric coordinates (0, 1 - w, w)
        float w = (d4 - d3)/((d4 - d3) + (d5 - d6));
        t[0] = 0.0f;
        t[1] = 1.0f - w;
        pt = pb + (pc - pb)*w;
        return (x - pt).norm();
    }

    // x inside face region. Compute pt through its barycentric coordinates (u, v, w)
    float denom = 1.0f/(va + vb + vc);
    float v = vb*denom;
    float w = vc*denom;
    t[0] = 1.0f - v - w;
    t[1] = v;

    pt = pa + ab*v + ac*w; //= u*a + v*b + w*c, u = va*denom = 1.0f - v - w
    return (x - pt).norm();
}

template<size_t DIM>
inline int Triangle<DIM>::intersect(const BoundingSphere<3>& s, std::vector<Interaction<3>>& is,
                               bool recordOneHit) const
{
    is.clear();
    const Vector<DIM>& pa = soup->positions[indices[0]];
    const Vector<DIM>& pb = soup->positions[indices[1]];
    const Vector<DIM>& pc = soup->positions[indices[2]];

    Interaction<3> i;
    float d = findClosestPointTriangle(pa, pb, pc, s.c, i.p, i.uv);

    if (d*d <= s.r2) {
        auto it = is.emplace(is.end(), Interaction<3>());
        it->primitiveIndex = pIndex;
        if (recordOneHit) {
            it->d = surfaceArea();

        } else {
            it->d = 1.0f;
        }

        return 1;
    }

    return 0;
}

template<size_t DIM>
inline bool Triangle<DIM>::findClosestPoint(BoundingSphere<DIM>& s, Interaction<DIM>& i, bool recordNormal) const
{
    const Vector<DIM>& pa = soup->positions[indices[0]];
    const Vector<DIM>& pb = soup->positions[indices[1]];
    const Vector<DIM>& pc = soup->positions[indices[2]];

    float d = findClosestPointTriangle(pa, pb, pc, s.c, i.p, i.uv);

    if (d*d <= s.r2) {
        i.d = d;
        i.primitiveIndex = pIndex;

        return true;
    }

    return false;
}

} // namespace fcpw
