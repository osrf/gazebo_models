varying vec4 vertex_depth;

uniform sampler2D tex;
varying vec2 uv;

void main()
{
  if (texture2D(tex, uv).a < 0.5)
  {
    discard;
  }

  float depth = (vertex_depth.z) / vertex_depth.w;

  gl_FragColor = vec4(depth, depth, depth, 1.0);
}
