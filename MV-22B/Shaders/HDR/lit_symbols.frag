$FG_GLSL_VERSION

// Shaders/HDR/lit.frag for a Canvas drawn over a painted face, for Models/Effects/Lit/symbols.eff: its light as there, and
// where it gives none no surface at all, so the face below stays as it is (Canvas textures have no alpha to cut it with).

in VS_OUT {
    float flogz;
    vec2 texcoord;
    vec3 vertex_normal;
    vec4 material_color;
} fs_in;

uniform sampler2D color_tex;

uniform float pbr_metallic;
uniform float pbr_roughness;
uniform vec3 lit_color;
uniform float lit_level;
uniform float lit_gain;

const float HDR_CD = 635.0;
const float DARK = 0.004;      // linear: below this the Canvas is its black background

// gbuffer_pack.glsl
void gbuffer_pack_pbr_opaque(vec3 normal,
                             vec3 base_color,
                             float metallic,
                             float roughness,
                             float occlusion,
                             vec3 emissive);
// color.glsl
vec3 eotf_inverse_sRGB(vec3 srgb);
// logarithmic_depth.glsl
float logdepth_encode(float z);

void main()
{
    // the brightest of a 3 x 3 grid across the pixel, as lit.frag, so thin strokes keep their luminance
    vec2 du = dFdx(fs_in.texcoord) / 3.0, dv = dFdy(fs_in.texcoord) / 3.0;
    vec3 lit = vec3(0.0);
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            lit = max(lit, eotf_inverse_sRGB(textureGrad(color_tex, fs_in.texcoord + float(i) * du + float(j) * dv, du, dv).rgb));
    if (max(lit.r, max(lit.g, lit.b)) < DARK)
        discard;
    vec3 emissive = lit * eotf_inverse_sRGB(lit_color) * max(lit_level * lit_gain, 0.0) / HDR_CD;
    vec3 N = normalize(fs_in.vertex_normal);
    gbuffer_pack_pbr_opaque(N, vec3(0.0), pbr_metallic, pbr_roughness, 1.0, emissive);
    gl_FragDepth = logdepth_encode(fs_in.flogz);
}
