// Renders the robot camera's grayscale image and runs the real AprilTag detector (vendor/apriltag-wasm.js) on it.
// Field frame and camera conventions follow localization.js. Pixel (i, j) covers [i, i+1) × [j, j+1), so its centre is at i + 0.5.
(function(root){
'use strict';
const L=root.FiducialLocalizer||(typeof require!=='undefined'?require('./localization.js'):null);
const DEG=Math.PI/180,IN=.0254,FIELD_HALF=72*IN,WALL_H=12*IN,TILE=24*IN;

function defaultEffects(){return {supersample:2,exposure:1,noise:3,defocus:0,speed:0,turnRateDegS:0,exposureMs:8,seed:1,ambient:.55}}

// Brown–Conrady radial model on normalized coordinates; k1 < 0 is barrel distortion as on most wide FTC webcams.
function distort(x,y,cam){let r2=x*x+y*y,f=1+(cam.k1||0)*r2+(cam.k2||0)*r2*r2;return [x*f,y*f]}
// Inverse by fixed-point iteration; null where the radial model has no inverse (outside the lens image circle).
function undistort(xd,yd,cam){let x=xd,y=yd;if(!cam.k1&&!cam.k2)return [x,y];for(let i=0;i<30;i++){let r2=x*x+y*y,f=1+(cam.k1||0)*r2+(cam.k2||0)*r2*r2;if(f<=.05)return null;x=xd/f;y=yd/f}
 let [cx,cy]=distort(x,y,cam);return Math.hypot(cx-xd,cy-yd)<1e-7?[x,y]:null}

// Scene primitives are built once per frame. Each has a screen bounding box so most samples skip it cheaply.
function buildScene({tags,patterns,polygons=[],cylinders=[]}){let prims=[];
 for(const tag of tags){let pattern=patterns&&patterns(tag.id);if(!pattern)continue;
  let {n,r,u}=L.tagFrame(tag),outer=tag.size*pattern.width/(pattern.width-2),c={x:tag.x+n.x*.001,y:tag.y+n.y*.001,z:tag.z+n.z*.001};
  prims.push({kind:'tag',c,n,r,up:u,half:outer/2,pattern,box:quadPoints(c,r,u,outer/2)})}
 for(const poly of polygons){let p=poly.points,e1=sub(p[1],p[0]),e2=sub(p[2],p[0]),n=norm(cross(e1,e2));prims.push({kind:'poly',p,n,d:dot(n,p[0]),albedo:poly.albedo,box:p})}
 for(const cyl of cylinders){let axis=sub(cyl.b,cyl.a),len=Math.hypot(axis.x,axis.y,axis.z);prims.push({kind:'cyl',a:cyl.a,axis:{x:axis.x/len,y:axis.y/len,z:axis.z/len},len,radius:cyl.radius,albedo:cyl.albedo,box:[cyl.a,cyl.b]})}
 return prims}
const sub=(a,b)=>({x:a.x-b.x,y:a.y-b.y,z:a.z-b.z}),dot=(a,b)=>a.x*b.x+a.y*b.y+a.z*b.z,cross=(a,b)=>({x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x}),norm=a=>{let l=Math.hypot(a.x,a.y,a.z);return {x:a.x/l,y:a.y/l,z:a.z/l}};
function quadPoints(c,r,u,h){return [[-1,-1],[1,-1],[1,1],[-1,1]].map(([i,j])=>({x:c.x+(r.x*i+u.x*j)*h,y:c.y+(r.y*i+u.y*j)*h,z:c.z+(r.z*i+u.z*j)*h}))}
const LIGHT=norm({x:.3,y:.5,z:1});

// Albedo of what a world ray hits (0–1), or the dim arena background.
function shade(O,D,prims,active){let best=Infinity,albedo=.16+.06*Math.max(0,D.z),hitN=null;
 if(D.z<0){let t=-O.z/D.z;if(t<best){let x=O.x+D.x*t,y=O.y+D.y*t;best=t;hitN=null;
  if(Math.abs(x)<=FIELD_HALF&&Math.abs(y)<=FIELD_HALF){let gx=Math.abs(((x+FIELD_HALF)%TILE+TILE)%TILE-TILE/2),gy=Math.abs(((y+FIELD_HALF)%TILE+TILE)%TILE-TILE/2),seam=Math.max(gx,gy)>TILE/2-.004;
   let ix=Math.floor((x+FIELD_HALF)/TILE),iy=Math.floor((y+FIELD_HALF)/TILE);albedo=seam?.18:.34+.03*(((ix*7+iy*13)%5)/4-.5)}else albedo=.1}}
 for(const [ax,s] of [['x',1],['x',-1],['y',1],['y',-1]]){let o=O[ax],d=D[ax];if(Math.abs(d)<1e-12)continue;let t=(s*FIELD_HALF-o)/d;if(t<=1e-6||t>=best)continue;
  let p={x:O.x+D.x*t,y:O.y+D.y*t,z:O.z+D.z*t},other=ax==='x'?p.y:p.x;if(p.z<0||p.z>WALL_H||Math.abs(other)>FIELD_HALF)continue;best=t;albedo=.62;hitN={x:ax==='x'?-s:0,y:ax==='y'?-s:0,z:0}}
 for(let m=0;m<active.count;m++){let pr=prims[active[m]];
  if(pr.kind==='tag'){let den=dot(D,pr.n);if(den>=0)continue;let t=dot(sub(pr.c,O),pr.n)/den;if(t<=0||t>=best)continue;let q=sub({x:O.x+D.x*t,y:O.y+D.y*t,z:O.z+D.z*t},pr.c),u=dot(q,pr.r)/pr.half,v=dot(q,pr.up)/pr.half;
   if(Math.abs(u)>1||Math.abs(v)>1)continue;let w=pr.pattern.width,i=Math.min(w-1,Math.floor((u+1)/2*w)),j=Math.min(w-1,Math.floor((1-v)/2*w));best=t;hitN=pr.n;albedo=pr.pattern.data[j*w+i]?.9:.05}
  else if(pr.kind==='poly'){let den=dot(D,pr.n);if(Math.abs(den)<1e-12)continue;let t=(pr.d-dot(O,pr.n))/den;if(t<=1e-6||t>=best)continue;let p={x:O.x+D.x*t,y:O.y+D.y*t,z:O.z+D.z*t},inside=true,sign=0;
   for(let i=0;i<pr.p.length&&inside;i++){let a=pr.p[i],b=pr.p[(i+1)%pr.p.length],s=dot(cross(sub(b,a),sub(p,a)),pr.n);if(Math.abs(s)<1e-12)continue;if(!sign)sign=Math.sign(s);else if(Math.sign(s)!==sign)inside=false}
   if(inside){best=t;hitN=pr.n;albedo=pr.albedo}}
  else{let w=sub(O,pr.a),dd=dot(D,pr.axis),dw=dot(w,pr.axis),A=1-dd*dd,B=dot(D,w)-dd*dw,C=dot(w,w)-dw*dw-pr.radius*pr.radius,disc=B*B-A*C;if(A<1e-12||disc<0)continue;
   let t=(-B-Math.sqrt(disc))/A;if(t<=1e-6||t>=best)continue;let s=dw+dd*t;if(s<0||s>pr.len)continue;best=t;albedo=pr.albedo;let p=sub({x:O.x+D.x*t,y:O.y+D.y*t,z:O.z+D.z*t},pr.a);hitN=norm(sub(p,{x:pr.axis.x*s,y:pr.axis.y*s,z:pr.axis.z*s}))}}
 return {albedo,lambert:hitN?Math.abs(dot(hitN,LIGHT)):LIGHT.z}}

function rng(seed){let s=seed>>>0||1;const next=()=>{s=s+0x6D2B79F5>>>0;let t=s;t=Math.imul(t^t>>>15,t|1);t^=t+Math.imul(t^t>>>7,t|61);return ((t^t>>>14)>>>0)/4294967296};
 return ()=>{let u=Math.max(next(),1e-12),v=next();return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v)}}

// Renders an 8-bit grayscale frame. Motion blur averages sub-frames over the exposure along the robot's motion.
function render({robot,camera,tags=L.defaultTags(),patterns,polygons,cylinders,effects={}}){
 let cam={...L.defaultCamera(),...camera},fx={...defaultEffects(),...effects},K=L.intrinsics(cam),W=cam.imageWidth,H=cam.imageHeight,ss=Math.max(1,fx.supersample|0);
 let prims=buildScene({tags,patterns,polygons,cylinders}),acc=new Float32Array(W*H),exposure=fx.exposureMs/1000;
 let sweepPx=K.fx*(Math.abs(fx.turnRateDegS)*DEG*exposure)+K.fx*Math.abs(fx.speed)*exposure/1.5,subframes=sweepPx<.25?1:Math.min(16,Math.ceil(2*sweepPx)+1);if(subframes>=4)ss=1;
 for(let f=0;f<subframes;f++){let s=subframes>1?(f/(subframes-1)-.5)*exposure:0,h=robot.headingDeg*DEG;
  let pose=L.cameraPose({x:robot.x+Math.cos(h)*fx.speed*s,y:robot.y+Math.sin(h)*fx.speed*s,headingDeg:robot.headingDeg+fx.turnRateDegS*s},cam);
  // Screen boxes of the primitives for this sub-frame; a primitive partly behind the camera is always tested.
  let boxes=prims.map(pr=>{let x0=Infinity,y0=Infinity,x1=-Infinity,y1=-Infinity;for(const P of pr.box){let c=L.toCamera(P,pose);if(c.z<.02)return null;let [dx,dy]=distort(c.x/c.z,c.y/c.z,cam);
   x0=Math.min(x0,dx*K.fx+K.cx);x1=Math.max(x1,dx*K.fx+K.cx);y0=Math.min(y0,dy*K.fy+K.cy);y1=Math.max(y1,dy*K.fy+K.cy)}let m=pr.kind==='cyl'?K.fx*pr.radius/.05+4:4;return [x0-m,y0-m,x1+m,y1+m]});
  let active=new Int32Array(prims.length);
  for(let j=0;j<H;j++){let rowActive=prims.map((_,k)=>k).filter(k=>!boxes[k]||boxes[k][1]<=j+1&&boxes[k][3]>=j);
   for(let i=0;i<W;i++){let n=0,sum=0;for(const k of rowActive)if(!boxes[k]||boxes[k][0]<=i+1&&boxes[k][2]>=i)active[n++]=k;active.count=n;
    for(let a=0;a<ss;a++)for(let b=0;b<ss;b++){let ray=undistort((i+(a+.5)/ss-K.cx)/K.fx,(j+(b+.5)/ss-K.cy)/K.fy,cam);if(!ray)continue;let [xn,yn]=ray;
     let D=norm({x:pose.f.x+xn*pose.r.x+yn*pose.d.x,y:pose.f.y+xn*pose.r.y+yn*pose.d.y,z:pose.f.z+xn*pose.r.z+yn*pose.d.z}),hit=shade(pose.C,D,prims,active);
     sum+=hit.albedo*(fx.ambient+(1-fx.ambient)*hit.lambert)}
    acc[j*W+i]+=sum/(ss*ss)}}}
 for(let k=0;k<W*H;k++)acc[k]/=subframes;let radiance=fx.defocus>0?gaussianBlur(acc,W,H,fx.defocus):acc;
 return {radiance,gray:sensor(radiance,fx,fx.seed),width:W,height:H,subframes,camera:cam,effects:fx}}
// Sensor stage: exposure gain, Gaussian read noise and 8-bit quantization. Cheap, so extra frames reuse one rendering.
function sensor(radiance,fx,seed){let gauss=rng(seed),gray=new Uint8ClampedArray(radiance.length);for(let k=0;k<radiance.length;k++)gray[k]=Math.round(255*fx.exposure*radiance[k]+fx.noise*gauss());return gray}

function gaussianBlur(src,W,H,sigma){let r=Math.ceil(3*sigma),k=[];for(let i=-r;i<=r;i++)k.push(Math.exp(-i*i/(2*sigma*sigma)));let s=k.reduce((a,b)=>a+b);k=k.map(v=>v/s);
 let tmp=new Float32Array(W*H),out=new Float32Array(W*H);
 for(let j=0;j<H;j++)for(let i=0;i<W;i++){let v=0;for(let d=-r;d<=r;d++)v+=k[d+r]*src[j*W+Math.min(W-1,Math.max(0,i+d))];tmp[j*W+i]=v}
 for(let j=0;j<H;j++)for(let i=0;i<W;i++){let v=0;for(let d=-r;d<=r;d++)v+=k[d+r]*tmp[Math.min(H-1,Math.max(0,j+d))*W+i];out[j*W+i]=v}return out}

// Wraps the compiled AprilRobotics detector. Its corners wrap counter-clockwise in the image starting at the tag's
// bottom-left, which matches localization.js's order (bottom-left, bottom-right, top-right, top-left).
function createDetector(Module){Module._at_init();let cache=new Map();
 return {
  tagCount:Module._at_tag_count(),
  pattern(id){if(cache.has(id))return cache.get(id);let out=Module._malloc(256),w=Module._at_tag_image(id,out),p=w?{width:w,data:Module.HEAPU8.slice(out,out+w*w)}:null;Module._free(out);cache.set(id,p);return p},
  detect(gray,width,height,{decimate=2,sigma=0,refineEdges=true,sharpening=.25}={}){Module._at_config(decimate,sigma,refineEdges?1:0,sharpening);
   let ptr=Module._at_buffer(width,height);Module.HEAPU8.set(gray,ptr);let n=Module._at_detect(width,height),base=Module._at_results()/8,r=Module.HEAPF64,out=[];
   for(let k=0;k<n;k++){let o=base+k*13;out.push({id:r[o],hamming:r[o+1],decisionMargin:r[o+2],center:{u:r[o+3],v:r[o+4]},corners:[0,1,2,3].map(c=>({u:r[o+5+2*c],v:r[o+6+2*c]}))})}
   return out}}}

// Full pixel pipeline: render, detect, keep trusted decodes, undistort corners with the known calibration and estimate
// the pose; with several frames (robot static, fresh sensor noise each) the estimates are fused by the Kalman filter.
// Corner bias from rendering and the detector's edge fit (about 0.2–0.3 px here) is common to all frames, so it is
// added to the covariance after fusion instead of averaging away.
function localizeFromImage({detector,robot,camera,tags=L.defaultTags(),layouts=[{tags}],polygons,cylinders,effects,detectorOptions,frames=1,maxHamming=0,minDecisionMargin=10,cornerBiasPx=.3}){
 let t0=now(),frame=render({robot,camera,tags,patterns:id=>detector.pattern(id),polygons,cylinders,effects}),renderMs=now()-t0,cam=frame.camera,K=L.intrinsics(cam),known=new Set(layouts[0].tags.map(t=>t.id));
 let filter=L.createPoseFilter(),last=null,framesUsed=0,detectMs=0,bias=null,chosen=null;
 for(let k=0;k<Math.max(1,frames|0);k++){let gray=k?sensor(frame.radiance,frame.effects,frame.effects.seed+k*7919):frame.gray,t1=now(),raw=detector.detect(gray,frame.width,frame.height,detectorOptions),accepted=[],rejected=[];detectMs+=now()-t1;
  for(const d of raw){let reason=!known.has(d.id)?'unknown':d.hamming>maxHamming?'hamming':d.decisionMargin<minDecisionMargin?'margin':null;if(reason){rejected.push({id:d.id,reason,decisionMargin:d.decisionMargin,corners:d.corners});continue}
   let corners=d.corners.map(q=>undistort((q.u-K.cx)/K.fx,(q.v-K.cy)/K.fy,cam));if(corners.some(c=>!c)){rejected.push({id:d.id,reason:'margin',decisionMargin:d.decisionMargin,corners:d.corners});continue}
   accepted.push({...d,corners:corners.map(([x,y])=>({u:x*K.fx+K.cx,v:y*K.fy+K.cy})),rawCorners:d.corners})}
  let e;if(!chosen){let best=L.estimateBest(accepted,{camera:cam,layouts,minPixelSigma:cornerBiasPx});chosen=best&&best.layout;e=best&&best.estimate}
  else e=L.estimate(accepted,{camera:cam,tags:chosen.tags,minPixelSigma:cornerBiasPx});if(e){filter.update({...e,covariance:e.covariance.map((row,i)=>row.map((v,j)=>v-e.biasCovariance[i][j]))});framesUsed++;if(!bias)bias=e.biasCovariance}if(!k)last={gray,detections:accepted,rejected,estimate:e}}
 let fused=filter.state,covariance=fused&&fused.covariance.map((row,i)=>row.map((v,j)=>v+bias[i][j]));
 let estimate=fused&&{...last.estimate,x:fused.x,y:fused.y,headingDeg:fused.headingDeg,covariance,std:{x:Math.sqrt(covariance[0][0]),y:Math.sqrt(covariance[1][1]),headingDeg:Math.sqrt(covariance[2][2])/DEG}};
 if(estimate&&!last.estimate)estimate={...estimate,rmsPx:NaN,tagIds:[],outlierTagIds:[]};
 return {frame:{...frame,gray:last.gray},hiveUp:chosen&&chosen.hiveUp,detections:last.detections,rejected:last.rejected,estimate,framesUsed,camera:cam,cameraPose:L.cameraPose(robot,cam),timing:{renderMs,detectMs:detectMs/Math.max(1,frames|0)},
  error:estimate?{x:estimate.x-robot.x,y:estimate.y-robot.y,position:Math.hypot(estimate.x-robot.x,estimate.y-robot.y),headingDeg:((estimate.headingDeg-robot.headingDeg+540)%360)-180}:null}}
const now=()=>typeof performance!=='undefined'?performance.now():Date.now();

const api={defaultEffects,distort,undistort,render,sensor,createDetector,localizeFromImage,FIELD_HALF,WALL_H};
if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.CameraRender=api;
})(typeof window!=='undefined'?window:globalThis);
