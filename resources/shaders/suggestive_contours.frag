#version 400

// IN: computed values from vertex shader
in float ndotv;
in float t_kr;
in float t_dwkr;
in vec3 view;
in vec3 w;
in float k;


in vec3 normal_test;
in vec3 pdir1_test;
in vec3 pdir2_test;
in float curv1_test;
in float curv2_test;
in vec4 dcurv_test;

// IN: uniform values
uniform bool jeroenmethod;
uniform float fz;
uniform float c_limit;
uniform float sc_limit;
uniform float dwkr_limit;

layout( location = 0 ) out vec4 frag_color;

void main()
{
        // base color
        vec4 color = vec4(1.0, 1.0, 1.0, 1.0);

        // use feature size
        float kr = fz*t_kr; // absolute value to use it in limits
        float dwkr = fz*fz*t_dwkr; // two times fz because derivative
        float dwkr_theta = (dwkr-dwkr*pow(ndotv, 2.0))/ length(w);

        // compute limits
        float contour_limit = c_limit*(pow(ndotv, 2.0)/abs(kr));

        float suggestive_contour_limit;
        if(jeroenmethod){
           suggestive_contour_limit = sc_limit*(abs(kr)/dwkr_theta);
        } else {
           suggestive_contour_limit = sc_limit*(dwkr);
        }

        // contours
        if(contour_limit < 1.0)
        {
           color.xyz = vec3(1,0,0);//min(color.xyz, vec3(0, 0, 0));
        }
        // suggestive contours
        else if((suggestive_contour_limit<1.0) && dwkr_theta>dwkr_limit)
        {
            color.xyz = vec3(0,0,1);//min(color.xyz, vec3(0, 0, 0));r
        }
        else
        {
            color.xyz = vec3(0,0,0);
            //color = vec4(0);
        }

        if(abs(k) < 0.0)
        {
            color.xyz = vec3(0,1,0);
        }


        //frag_color = vec4(vec3(ndotv),1);//color;
        frag_color = color;

}
