$FG_GLSL_VERSION

// Shaders/HDR/model_default.frag with emission: the texture times lit_color (sRGB), lit_level and lit_gain (cd/m2), for
// Models/Effects/Lit/lit.eff. The pipeline's unit of radiance is 635 cd/m2 (HDR_CD). The
// light on it is reflected as the texture times lit_reflectance (a lamp's dead front; 1 for integrally lit markings).

in VS_OUT {
    float flogz;
    vec2 texcoord;
    vec3 vertex_normal;
    vec4 material_color;
} fs_in;

uniform sampler2D color_tex;

uniform float alpha_cutoff;
uniform float pbr_metallic;
uniform float pbr_roughness;
uniform vec3 lit_color;
uniform float lit_level;
uniform float lit_gain;
uniform float lit_reflectance;

const float HDR_CD = 635.0;

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
    vec4 texel = texture(color_tex, fs_in.texcoord);
    if (texel.a * fs_in.material_color.a < alpha_cutoff)
        discard;
    vec3 texel_linear = eotf_inverse_sRGB(texel.rgb);
    vec3 color = texel_linear * fs_in.material_color.rgb * lit_reflectance;
    // the light it gives: the brightest of a 3 x 3 grid across the pixel, each tap a third of it wide. A lamp's legend
    // has strokes the eye resolves from the seat (0.6 mm at a metre is 2'), but that the screen's pixels don't (at a
    // 90 deg view a pixel is a millimetre there): averaged into the black round them, they'd be dim; shown at their
    // own luminance they look as they do to the eye. Where the texture is resolved, this is its texel.
    vec2 du = dFdx(fs_in.texcoord) / 3.0, dv = dFdy(fs_in.texcoord) / 3.0;
    vec3 lit = vec3(0.0);
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            lit = max(lit, eotf_inverse_sRGB(textureGrad(color_tex, fs_in.texcoord + float(i) * du + float(j) * dv, du, dv).rgb));
    vec3 emissive = lit * eotf_inverse_sRGB(lit_color) * max(lit_level * lit_gain, 0.0) / HDR_CD;
    vec3 N = normalize(fs_in.vertex_normal);
    gbuffer_pack_pbr_opaque(N, color, pbr_metallic, pbr_roughness, 1.0, emissive);
    gl_FragDepth = logdepth_encode(fs_in.flogz);
}
