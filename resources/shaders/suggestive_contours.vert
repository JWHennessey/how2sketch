#version 400

layout (location = 0) in vec4 vertex;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec3 pdir1;
layout (location = 3) in vec3 pdir2;
layout (location = 4) in float curv1;
layout (location = 5) in float curv2;
layout (location = 6) in vec4 dcurv;


// IN: camera position (per render)
uniform vec3 cam_pos;
uniform mat4 mvMatrix;
uniform mat4 modelMatrix;
uniform mat4 projMatrix;
uniform mat4 normalMatrix;
uniform mat4 viewMatrix;

// OUT: variables for fragment shader
out float ndotv;
out float t_kr;
out float t_dwkr;
out vec3 view;
out vec3 w;
out float k;
//test

out vec3 normal_test;
out vec3 pdir1_test;
out vec3 pdir2_test;
out float curv1_test;
out float curv2_test;
out vec4 dcurv_test;

void main()
{
        normal_test = normal;
        pdir1_test = pdir1;
        pdir2_test = pdir2;
        curv1_test = curv1;
        curv2_test = curv2;
        dcurv_test = dcurv;

        // compute vector to cam
        vec4 cam_pos_neg = viewMatrix * vec4(cam_pos, 1);
        //cam_pos_neg.y = -cam_pos_neg.y;
        //cam_pos_neg.y = -cam_pos_neg.y;
        vec4 eye_vertex = mvMatrix * vertex;
        view = (cam_pos_neg.xyz / cam_pos_neg.w) - vec3(eye_vertex.xyz);  //cam_pos_neg - vec3(eye_vertex);

        vec4 eye_normal = normalize(normalMatrix * vec4(normal, 1));

        vec4 pdir1_eye = normalize(normalMatrix * vec4(pdir1, 0));
        vec4 pdir2_eye = normalize(normalMatrix * vec4(pdir2, 0));
        vec4 dcurv_eye = normalize(normalMatrix * dcurv);
        // compute ndotv (and divide by view)
        ndotv = (1.0 / length(view)) * dot(eye_normal.xyz, view);
        k = dot(pdir1, pdir2);
        // optimalisation: if this vector points away from cam, don't even bother computing the rest.
        // the data will not be used in computing pixel color
        if(!(ndotv < 0.0))
        {
                // compute kr
                w = normalize(view - eye_normal.xyz * dot(view, eye_normal.xyz));

                float u = dot(w, pdir1_eye.xyz);
                float v = dot(w, pdir2_eye.xyz);
                float u2 = u*u;
                float v2 = v*v;
                t_kr = (curv1*u2) + (curv2*v2);
                // and dwkr
                float uv = u*v;
                float dwII = (u2*u*dcurv_eye.x) + (3.0*u*uv*dcurv_eye.y) + (3.0*uv*v*dcurv_eye.z) + (v*v2*dcurv_eye.w);
                // extra term due to second derivative
                t_dwkr = dwII + 2.0 * curv1 * curv2 * ndotv/sqrt((1.0 - pow(ndotv, 2.0)));
        }

        // position transformation
        gl_Position = projMatrix * mvMatrix * vertex;

}
