// For more info on plugins, parameters and how to use them refer to the
// V-Ray AppSDK documentation.
#pragma warning(disable : 4996)

#define _USE_MATH_DEFINES
#include <cmath>

#define VRAY_RUNTIME_LOAD_PRIMARY
#include "vraysdk.hpp"
#include "vrayplugins.hpp"

using namespace VRay;
using namespace VRay::Plugins;

#include "utils.h"
using namespace std;

const char *BASE_PATH = getenv("VRAY_SDK");
string SCENE_PATH = (BASE_PATH ? string(BASE_PATH) : string(".")) + PATH_DELIMITER + "scenes";

// Container representing a box.
struct BBox
{
    Vector pmin; /// The lower bounds for the box along the three axes
    Vector pmax; /// The upper bounds for the box along the three axes
    BBox(Vector imin, Vector imax)
    {
        pmin = imin;
        pmax = imax;
    }
};
​
    // Container and tools for geometric data.
    struct TriMesh
{
    int numVerts;
    int numFaces;
    VectorList verts;
    IntList faces;
    VectorList vnormals;
    VectorList fnormals;
    FloatList farea2;
    VectorList uvw;
    ​ void setNextVert(Vector v) { verts.push_back(v); }
    void setNextFace(int v0 = 0, int v1 = 0, int v2 = 0)
    {
        faces.push_back(v0);
        faces.push_back(v1);
        faces.push_back(v2);
    }
    void setNextVNormal(Vector v) { vnormals.push_back(v); }
    void setNextFNormal(Vector v) { fnormals.push_back(v); }
    void setNextFArea2(float area) { farea2.push_back(area); }
    void setNextUVW(Vector v) { uvw.push_back(v); }
    // Convert from unit spherical to Cartesian coordinates.
    // theta - The elevation angle in the range [-pi/2, pi/2].
    // phi - The azimuth angle in the range [0, 2*pi].
    // tolerance - Tolerance towards 0 for verts
    // Returns the resulting point on the unit sphere.
    Vector getSphereCoords(float theta, float phi, float tolerance = 1e-6f)
    {
        float thetaCos = std::cosf(theta);
        float thetaSin = std::sinf(theta);
        return Vector(std::fabs(std::cosf(phi) * thetaCos) > tolerance ? std::cosf(phi) * thetaCos : 0.0f,
                      std::fabs(std::sinf(phi) * thetaCos) > tolerance ? std::sinf(phi) * thetaCos : 0.0f,
                      std::fabs(thetaSin) > tolerance ? thetaSin : 0.0f);
    }
    // Builds vertices normals and face normals.
    // buildVNormals - Whether should build normals for vertices
    void buildNormals(bool buildVNormals = true)
    {
        fnormals.reserve(numFaces);
        farea2.reserve(numFaces);
        ​ for (int i = 0; i < numFaces; i++)
        {
            Vector p0 = verts[faces[i * 3 + 0]];
            Vector p1 = verts[faces[i * 3 + 1]];
            Vector p2 = verts[faces[i * 3 + 2]];
            Vector n = (p1 - p0) ^ (p2 - p0);
            ​
                setNextFArea2(n.length());
            if (farea2[i] != 0.0f)
                setNextFNormal(n / farea2[i]);
            else
                setNextFNormal(Vector(0.0f, 0.0f, 0.0f));
        }
        ​ if (buildVNormals)
        {
            vnormals.reserve(numVerts);
            for (int i = 0; i < numVerts; i++)
                setNextVNormal(Vector(0.0f, 0.0f, 0.0f));
            for (int i = 0; i < numFaces; i++)
            {
                for (int j = 0; j < 3; j++)
                    vnormals[faces[i * 3 + j]] += fnormals[i] * farea2[i];
            }
            for (int i = 0; i < numVerts; i++)
            {
                float len = vnormals[i].length();
                if (len != 0.0f)
                    vnormals[i] /= len;
            }
        }
    }
    // Builds a disc TriMesh object
    // radius - Radius
    // subdivs - Subdivisions<
    // tolerance - Tolerance towards 0 for verts
    void makeDisc(float radius, int subdivs, float tolerance = 1e-6f)
    {
        numVerts = subdivs + 1;
        verts.reserve(numVerts);
        numFaces = subdivs;
        faces.reserve(numFaces * 3);
        setNextVert(Vector(0.0f, 0.0f, 0.0f));
        for (int i = 0; i < subdivs; i++)
        {
            float f = (float)i / subdivs;
            float sn = std::sinf(f * 2.0f * (float)M_PI);
            float cs = std::cosf(f * 2.0f * (float)M_PI);
            setNextVert(Vector(std::fabs(radius * cs) > tolerance ? radius * cs : 0.0f,
                               std::fabs(radius * sn) > tolerance ? radius * sn : 0.0f,
                               0.0f));
            ​ if (i + 1 == subdivs)
                setNextFace(0, i + 1, 1);
            else setNextFace(0, i + 1, i + 2);
        }
        buildNormals();
    }
    // Builds a cylinder TriMesh object
    // radius - Radius
    // height - Height
    // subdivs - Subdivisions
    // caps - If true, the cylinder has top and bottom
    // tolerance - Tolerance towards 0 for verts
    void makeCylinder(float radius, float height, int subdivs, bool caps = true, float tolerance = 1e-6f)
    {
        if (caps)
        {
            numVerts = (subdivs + 1) * 2;
            verts.reserve(numVerts);
            numFaces = subdivs * 4;
            faces.reserve(numFaces * 3);
            setNextVert(Vector(0.0f, 0.0f, -height * 0.5f));
            setNextVert(Vector(0.0f, 0.0f, height * 0.5f));
            for (int i = 0; i < subdivs; i++)
            {
                float f = (float)i / subdivs;
                float sn = (float)std::sinf(f * 2.0f * (float)M_PI);
                float cs = (float)std::cosf(f * 2.0f * (float)M_PI);
                setNextVert(Vector(std::fabs(radius * cs) > tolerance ? radius * cs : 0.0f,
                                   std::fabs(radius * sn) > tolerance ? radius * sn : 0.0f,
                                   -height * 0.5f));
                setNextVert(Vector(std::fabs(radius * cs) > tolerance ? radius * cs : 0.0f,
                                   std::fabs(radius * sn) > tolerance ? radius * sn : 0.0f,
                                   height * 0.5f));
                int v0 = 2 + i * 2;
                int v1 = 2 + (i + 1) * 2;
                int v2 = 2 + (i + 1) * 2 + 1;
                int v3 = 2 + i * 2 + 1;
                if (i + 1 == subdivs)
                {
                    v1 = 2;
                    v2 = 3;
                }
                setNextFace(0, v1, v0);
                setNextFace(v0, v1, v2);
                setNextFace(v2, v3, v0);
                setNextFace(1, v3, v2);
            }
        }
        else
        {
            numVerts = (subdivs)*2;
            verts.reserve(numVerts);
            numFaces = subdivs * 2;
            faces.reserve(numFaces * 3);
            for (int i = 0; i < subdivs; i++)
            {
                float f = (float)i / subdivs;
                float sn = (float)std::sinf(f * 2.0f * (float)M_PI);
                float cs = (float)std::cosf(f * 2.0f * (float)M_PI);
                setNextVert(Vector(std::fabs(radius * cs) > tolerance ? radius * cs : 0.0f,
                                   std::fabs(radius * sn) > tolerance ? radius * sn : 0.0f,
                                   -height * 0.5f));
                setNextVert(Vector(std::fabs(radius * cs) > tolerance ? radius * cs : 0.0f,
                                   std::fabs(radius * sn) > tolerance ? radius * sn : 0.0f,
                                   height * 0.5f));
                int v0 = i * 2;
                int v1 = (i + 1) * 2;
                int v2 = (i + 1) * 2 + 1;
                int v3 = i * 2 + 1;
                if (i + 1 == subdivs)
                {
                    v1 = 0;
                    v2 = 1;
                }
                setNextFace(v0, v1, v2);
                setNextFace(v2, v3, v0);
            }
        }
        ​
        buildNormals();
    }
    // Builds a sphere TriMesh object
    // radius - Radius
    // subdivs - Subdivisions
    // generateTexCoords - Whether should generate TexCoords
    // tolerance - Tolerance towards 0 for verts
    void makeSphere(float radius, int subdivs, bool generateTexCoords = false, float tolerance = 1e-6f)
    {
        numVerts = subdivs * (subdivs - 1) + 2;
        verts.reserve(numVerts);
        numFaces = subdivs * (subdivs - 1) * 2;
        faces.reserve(numFaces * 3);
        if (generateTexCoords)
            uvw.reserve(numVerts);
        vnormals.reserve(numVerts);
        // Pole
        Vector unitMinus = Vector(0.0f, 0.0f, -1.0f);
        setNextVert(unitMinus * radius);
        if (generateTexCoords)
            setNextUVW(Vector(0.0f, 0.0f, 0.0f));
        setNextVNormal(unitMinus);
        // Middle rows
        for (int i = 1; i < subdivs; i++)
        {
            for (int j = 0; j < subdivs; j++)
            {
                float u = (float)i / subdivs;
                float v = (float)j / subdivs;
                float fx = -1.0f + 2.0f * u;
                float fy = v;
                Vector unitDir = getSphereCoords(fx * (float)M_PI * 0.5f, fy * (float)M_PI * 2.0f, tolerance);
                setNextVert(unitDir * radius);
                if (generateTexCoords)
                    setNextUVW(Vector(u, v, 0.0f));
                setNextVNormal(unitDir);
            }
        }
        // Pole
        Vector unitPlus = Vector(0.0f, 0.0f, 1.0f);
        setNextVert(unitPlus * radius);
        if (generateTexCoords)
            setNextUVW(Vector(1.0f, 1.0f, 0.0f));
        setNextVNormal(unitPlus);
        ​
            // First row
            for (int i = 0; i < subdivs; i++)
        {
            if (i + 1 == subdivs)
                setNextFace(1, i + 1, 0);
            else
                setNextFace(i + 2, i + 1, 0);
        }
        // Middle rows
        for (int i = 0; i < (subdivs - 2); i++)
        {
            int aidx = i * subdivs + 1;
            int bidx = (i + 1) * subdivs + 1;
            for (int j = 0; j < subdivs; j++)
            {
                int fa = aidx + j;
                int fb = aidx + (j + 1) % subdivs;
                int fc = bidx + j;
                int fd = bidx + (j + 1) % subdivs;
                setNextFace(fa, fb, fd);
                setNextFace(fa, fd, fc);
            }
        }
        // Last row
        int lastVertIdx = numVerts - 1;
        for (int i = 0; i < subdivs; i++)
        {
            if (i + 1 == subdivs)
                setNextFace(lastVertIdx - 1, lastVertIdx - (i + 1), lastVertIdx);
            else
                setNextFace(lastVertIdx - (i + 2), lastVertIdx - (i + 1), lastVertIdx);
        }
        ​
            buildNormals(false);
    }
    // Builds a box TriMesh object
    // box - A box object
    void makeBox(BBox &box)
    {
        Vector p = box.pmin;
        Vector q = box.pmax;
        ​
            numVerts = 8;
        verts.reserve(numVerts);
        setNextVert(Vector(p.x, p.y, p.z));
        setNextVert(Vector(q.x, p.y, p.z));
        setNextVert(Vector(p.x, p.y, q.z));
        setNextVert(Vector(q.x, p.y, q.z));
        setNextVert(Vector(p.x, q.y, p.z));
        setNextVert(Vector(q.x, q.y, p.z));
        setNextVert(Vector(p.x, q.y, q.z));
        setNextVert(Vector(q.x, q.y, q.z));
        ​
            numFaces = 12;
        faces.reserve(numFaces * 3);
        setNextFace(0, 1, 2);
        setNextFace(1, 3, 2);
        setNextFace(4, 6, 5);
        setNextFace(5, 6, 7);
        setNextFace(1, 5, 3);
        setNextFace(5, 7, 3);
        setNextFace(0, 2, 4);
        setNextFace(4, 2, 6);
        setNextFace(2, 3, 6);
        setNextFace(3, 7, 6);
        setNextFace(0, 4, 1);
        setNextFace(1, 4, 5);
        ​
        buildNormals();
    }
};
​
    Transform
    makeTransform(float rotX = 0, float rotY = 0, float rotZ = 0, float scale = 1, Vector offset = Vector())
{
    auto mS = Matrix(scale);
    auto mX = makeRotationMatrixX(rotX);
    auto mY = makeRotationMatrixY(rotY);
    auto mZ = makeRotationMatrixZ(rotZ);
    auto transform = Transform();
    transform.matrix = mS * mZ * mY * mX;
    transform.offset = offset;
    return transform;
}
​ void setupScene(VRayRenderer &renderer, float objDiamenter, float lightAngleZ)
{
    auto renderView = renderer.newPlugin<RenderView>();
    ​
        Vector camPos(0.0f, -2.0 * objDiamenter, 0.0f);
    auto camTransform = makeTransform((float)M_PI / 2, 0.0f, 0.0f, 1.0f, camPos);
    renderView.set_transform(camTransform);
    ​
        //auto light = renderer.newPlugin<LightRectangle>();
        auto light = renderer.newPlugin<LightOmni>();
    auto adjust = makeTransform(0.0f, 0.0f, lightAngleZ, 1.0f, camPos);
    light.set_transform(adjust * camTransform);
    // light.set_u_size(0.5f * objDiamenter);
    // light.set_v_size(0.35f * objDiamenter);
    light.set_intensity(10);
}
​
    Plugin
    setupMaterial(VRayRenderer &renderer)
{
    BRDFVRayMtl brdf = renderer.newPlugin<BRDFVRayMtl>();
    ​
        brdf.set_diffuse(AColor(1, 0.854, 0.051, 1));
    brdf.set_fresnel(true);
    // brdf.set_metalness(1.0);
    // brdf.set_roughness(0.5);
    ​ auto material = renderer.newPlugin<MtlSingleBRDF>();
    material.set_brdf(brdf);
    ​ return material;
}
​ void addObj(VRayRenderer &renderer, TriMesh &mesh, Plugin &material, bool smooth = false)
{
    GeomStaticMesh geometry = renderer.newPlugin<GeomStaticMesh>();
    geometry.set_vertices(mesh.verts);
    geometry.set_faces(mesh.faces);
    geometry.set_normals(mesh.vnormals);
    ​
        GeomStaticSmoothedMesh smoothed;
    if (smooth)
    {
        smoothed = renderer.newPlugin<GeomStaticSmoothedMesh>();
        smoothed.set_mesh(geometry);
    }
    ​ auto node = renderer.newPlugin<Node>();
    node.set_material(material);
    node.set_geometry(smooth ? (Plugin)smoothed : geometry);
    node.set_transform(makeTransform((float)M_PI / 2, 0, 0));
}
​ void logMessage(VRay::VRayRenderer &renderer, const char *message, VRay::MessageLevel level, double instant, void *userData)
{
    switch (level)
    {
    case VRay::MessageError:
        printf("[ERROR] %s\n", message);
        break;
    case VRay::MessageWarning:
        printf("[Warning] %s\n", message);
        break;
    case VRay::MessageInfo:
        printf("[info] %s\n", message);
        break;
    case VRay::MessageDebug:
        // Uncomment for testing, but you might want to ignore these in real code
        //printf("[debug] %s\n", message);
        break;
    }
}
​ void renderSequence(int nSeq)
{
    VRayRenderer renderer;
    renderer.setOnLogMessage(logMessage);
    ​ for (int i = 0; i <= nSeq; i++)
    {
        renderer.clearScene();
        ​ float r = 1.0f;
        int divs = 64;
        TriMesh newMesh;
        newMesh.makeSphere(r, divs);
        ​ auto material = setupMaterial(renderer);
        addObj(renderer, newMesh, material);
        ​
            setupScene(renderer, 2 * r, (float)M_PI / nSeq * i - (float)M_PI / 2);
        ​
            renderer.startSync();
        renderer.waitForRenderEnd();
        ​ auto image = renderer.getImage();
        image->saveToPng("output_" + to_string(i) + ".png");
        delete image;
    }
}
​ int main()
{
    VRayInit init(NULL, true);
    renderSequence(6);
    ​ return 0;
}