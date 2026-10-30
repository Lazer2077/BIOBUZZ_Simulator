// Simulated robot camera and AprilTag-style fiducial localization for the BIOBUZZ simulator.
// Field frame as in index.html: metres, X between the red and blue HIVEs, Y between the CELLs, Z up.
// Robot frame: x forward, y left. Camera frame (OpenCV): x right, y down, z forward.
// Angles are degrees at the API boundary and radians internally.
(function(root){
'use strict';
const DEG=Math.PI/180,IN=.0254;
// Competition Manual §9.9: 3.25 in tag36h11 tags.
const TAG_SIZE=3.25*IN;
const dot=(a,b)=>a.x*b.x+a.y*b.y+a.z*b.z,sub=(a,b)=>({x:a.x-b.x,y:a.y-b.y,z:a.z-b.z});
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));

function defaultCamera(){return {imageWidth:640,imageHeight:480,hfovDeg:70,mountHeight:.30,mountForward:0,mountLeft:0,pitchDeg:10,yawOffsetDeg:0,
 maxRange:4.5,minTagPixels:10,maxViewAngleDeg:75,edgeMargin:2}}
function intrinsics(cam){let fx=cam.imageWidth/2/Math.tan(cam.hfovDeg*DEG/2);return {fx,fy:fx,cx:cam.imageWidth/2,cy:cam.imageHeight/2}}

// Official layout (Competition Manual TU07 §9.9, Figures 9-15 to 9-17): each CELL carries one AprilTag Cluster of
// four tags on its bottom face (the outside of the back panel), facing away from the opening, with the sticker's
// bottom edge toward the field centre. Tag centres sit ±2.75 in and ±6.5 in from the cluster centre along X, and
// 0.188 in from the back panel's middle toward its front (flat) edge: the reference holes are 9.938 in from the front
// edge and the tag centreline is 2.75 in above them.
// The simulator's +Y is the audience side: in Figure 9-17 the starting NECTAR (upward CELLs) are in the red audience
// CELL and the blue far CELL, which here are red +Y and blue −Y. That makes the simulator frame a mirror image of the
// field in Y, so the IDs follow Figure 9-17's left-to-right order along X, and each tag is drawn readable from below.
const HIVE_OFFSET=25.5*IN/2,CELL_OFFSET=.4075,PIVOT_Z=43.95*IN,OPENING_Z=1.5003,CELL_PITCH=Math.PI/3,CELL_DEPTH=12*IN,CLUSTER_V=-.188*IN;
const CLUSTER_IDS={red:{'-1':[33,32,31,30],'1':[34,35,36,37]},blue:{'-1':[45,44,43,42],'1':[38,39,40,41]}},CLUSTER_X=[-6.5,-2.75,2.75,6.5].map(v=>v*IN);
// A CELL's frame in its current pose, as index.html's cellGeometry/restCellGeometry: origin at the opening centre,
// normal out of the opening, tangent along the opening from the flat edge toward the apex.
function cellFrame(name,s,up){let x=name==='red'?-HIVE_OFFSET:HIVE_OFFSET,o={x,y:s*CELL_OFFSET,z:OPENING_Z},n={x:0,y:s*Math.sin(CELL_PITCH),z:Math.cos(CELL_PITCH)},t={x:0,y:-s*Math.cos(CELL_PITCH),z:Math.sin(CELL_PITCH)};
 if(s===up)return {origin:o,normal:n,tangent:t};let phi=s*Math.PI/3,co=Math.cos(phi),si=Math.sin(phi),v=o.z-PIVOT_Z,dir=d=>({x:d.x,y:d.y*co+d.z*si,z:d.z*co-d.y*si});
 return {origin:{x,y:o.y*co+v*si,z:PIVOT_Z+v*co-o.y*si},normal:dir(n),tangent:dir(t)}}
// All 16 tags for given upward sides ({red: ±1, blue: ±1}); the official start is red +1, blue −1.
function defaultTags(hiveUp={red:1,blue:-1}){let tags=[];
 for(const name of ['red','blue'])for(const s of [-1,1]){let f=cellFrame(name,s,hiveUp[name]),c={x:f.origin.x,y:f.origin.y+f.tangent.y*CLUSTER_V-f.normal.y*(CELL_DEPTH+.002),z:f.origin.z+f.tangent.z*CLUSTER_V-f.normal.z*(CELL_DEPTH+.002)};
  let normal={x:-f.normal.x,y:-f.normal.y,z:-f.normal.z},up={x:-f.tangent.x,y:-f.tangent.y,z:-f.tangent.z};
  CLUSTER_IDS[name][s].forEach((id,k)=>tags.push({id,hive:name,side:s,cluster:`${name}${s>0?'+':'-'}`,x:c.x+CLUSTER_X[k],y:c.y,z:c.z,normal,up,size:TAG_SIZE}))}
 return tags.sort((a,b)=>a.id-b.id)}
// The four combinations of HIVE poses, for when the robot must tell from the tags which way each HIVE is tipped.
function hiveLayouts(){return [1,-1].flatMap(r=>[1,-1].map(b=>({hiveUp:{red:r,blue:b},tags:defaultTags({red:r,blue:b})})))}

// Tag frame: outward normal n, up u (toward the tag's top edge) and right r = u × n as seen from in front.
// Tags without explicit vectors are vertical, facing yawDeg.
function tagFrame(tag){let n=tag.normal,u=tag.up;if(!n){let a=tag.yawDeg*DEG;n={x:Math.cos(a),y:Math.sin(a),z:0};u={x:0,y:0,z:1}}
 return {n,u,r:{x:u.y*n.z-u.z*n.y,y:u.z*n.x-u.x*n.z,z:u.x*n.y-u.y*n.x}}}
// Corners in the order bottom-left, bottom-right, top-right, top-left as seen from in front of the tag.
function tagCorners(tag){let {u,r}=tagFrame(tag),h=tag.size/2;
 return [[-1,-1],[1,-1],[1,1],[-1,1]].map(([i,j])=>({x:tag.x+(i*r.x+j*u.x)*h,y:tag.y+(i*r.y+j*u.y)*h,z:tag.z+(i*r.z+j*u.z)*h}))}
function tagNormal(tag){return tagFrame(tag).n}

function cameraPose(robot,cam){let th=robot.headingDeg*DEG,psi=th+cam.yawOffsetDeg*DEG,p=cam.pitchDeg*DEG,cp=Math.cos(p),sp=Math.sin(p);
 return {C:{x:robot.x+Math.cos(th)*cam.mountForward-Math.sin(th)*cam.mountLeft,y:robot.y+Math.sin(th)*cam.mountForward+Math.cos(th)*cam.mountLeft,z:cam.mountHeight},
  f:{x:Math.cos(psi)*cp,y:Math.sin(psi)*cp,z:sp},r:{x:Math.sin(psi),y:-Math.cos(psi),z:0},d:{x:sp*Math.cos(psi),y:sp*Math.sin(psi),z:-cp}}}
function toCamera(P,pose){let q=sub(P,pose.C);return {x:dot(q,pose.r),y:dot(q,pose.d),z:dot(q,pose.f)}}
function project(P,pose,K){let c=toCamera(P,pose);if(c.z<1e-3)return null;return {u:K.fx*c.x/c.z+K.cx,v:K.fy*c.y/c.z+K.cy,depth:c.z}}

function rng(seed){let s=seed>>>0||1;const next=()=>{s=s+0x6D2B79F5>>>0;let t=s;t=Math.imul(t^t>>>15,t|1);t^=t+Math.imul(t^t>>>7,t|61);return ((t^t>>>14)>>>0)/4294967296};
 return {next,gauss(){let u=Math.max(next(),1e-12),v=next();return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v)}}}

// Simulate the tag detector: which tags the camera sees and their noisy corner pixels.
function detect({robot,camera=defaultCamera(),tags=defaultTags(),pixelNoise=.5,seed=1,dropout=0,occluded=null}={}){
 let cam={...defaultCamera(),...camera},K=intrinsics(cam),pose=cameraPose(robot,cam),random=rng(seed),detections=[],rejected=[];
 for(const tag of tags){let corners=tagCorners(tag),centre={x:tag.x,y:tag.y,z:tag.z},toCam=sub(pose.C,centre),dist=Math.hypot(toCam.x,toCam.y,toCam.z),reason=null,ideal=null;
  if(toCamera(centre,pose).z<=0)reason='behind';
  else if(dist>cam.maxRange)reason='range';
  else if(dot(tagNormal(tag),toCam)/dist<=Math.cos(cam.maxViewAngleDeg*DEG))reason='angle';
  else{ideal=corners.map(P=>project(P,pose,K));let m=cam.edgeMargin;
   if(ideal.some(q=>!q||q.u<m||q.v<m||q.u>cam.imageWidth-m||q.v>cam.imageHeight-m))reason='fov';
   else if(Math.min(...ideal.map((q,i)=>{let n=ideal[(i+1)%4];return Math.hypot(n.u-q.u,n.v-q.v)}))<cam.minTagPixels)reason='small';
   else if(occluded&&occluded(pose.C,centre))reason='occluded';
   else if(dropout>0&&random.next()<dropout)reason='dropout'}
  if(reason){rejected.push({id:tag.id,reason});continue}
  detections.push({id:tag.id,corners:ideal.map(q=>({u:q.u+pixelNoise*random.gauss(),v:q.v+pixelNoise*random.gauss()})),ideal:ideal.map(q=>({u:q.u,v:q.v}))})}
 return {detections,rejected,camera:cam,cameraPose:pose}}

function solve3(A,b){let m=A.map((row,i)=>[...row,b[i]]);
 for(let c=0;c<3;c++){let p=c;for(let r=c+1;r<3;r++)if(Math.abs(m[r][c])>Math.abs(m[p][c]))p=r;if(Math.abs(m[p][c])<1e-15)return null;[m[c],m[p]]=[m[p],m[c]];
  for(let r=0;r<3;r++)if(r!==c){let f=m[r][c]/m[c][c];for(let k=c;k<4;k++)m[r][k]-=f*m[c][k]}}
 return m.map((row,i)=>row[3]/row[i])}
function inverse3(A){let cols=[0,1,2].map(j=>solve3(A,[0,1,2].map(i=>i===j?1:0)));return cols.every(Boolean)?[0,1,2].map(i=>cols.map(c=>c[i])):null}

// Initial pose for tags in any orientation. With the camera height and pitch known, fixing the heading fixes every
// corner's viewing ray in the field, and the camera position then follows from a linear least-squares fit of those
// rays through the known corners. The heading is scanned in 2° steps and the best bracket narrowed by golden-section
// search, which is exact without noise and needs only one tag.
function initialPose(obs,cam,K){let pts=[];for(const o of obs)o.corners.forEach((q,i)=>pts.push({P:o.world[i],xn:(q.u-K.cx)/K.fx,yn:(q.v-K.cy)/K.fy}));
 if(pts.length<4)return null;
 const fit=th=>{let pose=cameraPose({x:0,y:0,headingDeg:th/DEG},cam),A=[0,0,0],b=[0,0],eqs=[];
  for(const {P,xn,yn} of pts){let d={x:pose.f.x+xn*pose.r.x+yn*pose.d.x,y:pose.f.y+xn*pose.r.y+yn*pose.d.y,z:pose.f.z+xn*pose.r.z+yn*pose.d.z},l=Math.hypot(d.x,d.y,d.z);d={x:d.x/l,y:d.y/l,z:d.z/l};
   let q=sub(P,pose.C);for(const [a1,a2,rhs] of [[0,-d.z,q.z*d.y-q.y*d.z],[d.z,0,q.x*d.z-q.z*d.x],[-d.y,d.x,q.y*d.x-q.x*d.y]]){A[0]+=a1*a1;A[1]+=a1*a2;A[2]+=a2*a2;b[0]+=a1*rhs;b[1]+=a2*rhs;eqs.push([a1,a2,rhs])}
   }
  let det=A[0]*A[2]-A[1]*A[1];if(Math.abs(det)<1e-15)return {cost:Infinity};let x=(b[0]*A[2]-b[1]*A[1])/det,y=(A[0]*b[1]-A[1]*b[0])/det,cost=0;
  for(const [a1,a2,rhs] of eqs)cost+=(a1*x+a2*y-rhs)**2;
  // Reject headings that put the tags behind the camera.
  for(const {P} of pts){let c=toCamera(P,{...pose,C:{x:pose.C.x+x,y:pose.C.y+y,z:pose.C.z}});if(c.z<=0){cost+=1e3;break}}
  return {cost,x,y}};
 let best=0,bestCost=Infinity,step=2*DEG;for(let th=-Math.PI;th<Math.PI;th+=step){let c=fit(th).cost;if(c<bestCost){bestCost=c;best=th}}
 let a=best-step,b=best+step,g=(Math.sqrt(5)-1)/2;for(let k=0;k<60;k++){let c1=b-g*(b-a),c2=a+g*(b-a);if(fit(c1).cost<fit(c2).cost)b=c2;else a=c1}
 let th=(a+b)/2,f=fit(th);return Number.isFinite(f.cost)?[f.x,f.y,wrap(th)]:null}

// Levenberg–Marquardt on the corner reprojection error over (x, y, heading), with Huber weights.
function refine(obs,cam,K,start,maxIterations,huberPx){
 const residuals=s=>{let pose=cameraPose({x:s[0],y:s[1],headingDeg:s[2]/DEG},cam),r=[];
  for(const o of obs)o.world.forEach((P,i)=>{let q=project(P,pose,K);r.push(q?o.corners[i].u-q.u:1e3,q?o.corners[i].v-q.v:1e3)});return r};
 const weights=r=>{let w=[];for(let i=0;i<r.length;i+=2){let e=Math.hypot(r[i],r[i+1]),k=e<=huberPx?1:huberPx/e;w.push(k,k)}return w};
 const cost=(r,w)=>r.reduce((s,v,i)=>s+w[i]*v*v,0);
 const jacobian=s=>[1e-5,1e-5,1e-6].map((h,j)=>{let a=[...s],b=[...s];a[j]+=h;b[j]-=h;let ra=residuals(a),rb=residuals(b);return ra.map((v,i)=>-(v-rb[i])/(2*h))});
 let s=[...start],r=residuals(s),w=weights(r),c=cost(r,w),lambda=1e-3,it=0,done=false;
 for(;it<maxIterations&&!done;it++){let J=jacobian(s),A=[[0,0,0],[0,0,0],[0,0,0]],g=[0,0,0];
  for(let i=0;i<r.length;i++)for(let a=0;a<3;a++){g[a]-=J[a][i]*w[i]*r[i];for(let b=0;b<3;b++)A[a][b]+=J[a][i]*w[i]*J[b][i]}
  let improved=false;for(let tries=0;tries<8&&!improved;tries++){let step=solve3(A.map((row,i)=>row.map((v,j)=>i===j?v*(1+lambda):v)),g.map(v=>-v));if(!step)break;
   let next=[s[0]+step[0],s[1]+step[1],wrap(s[2]+step[2])],rn=residuals(next),wn=weights(rn),cn=cost(rn,wn);
   if(cn<c){improved=true;let small=c-cn<1e-9*c+1e-18||Math.hypot(step[0],step[1])<1e-7&&Math.abs(step[2])<1e-7;s=next;r=rn;w=wn;c=cn;lambda=Math.max(lambda/10,1e-9);done=small}else lambda*=10}
  if(!improved)break}
 let J=jacobian(s),A=[[0,0,0],[0,0,0],[0,0,0]];for(let i=0;i<r.length;i++)for(let a=0;a<3;a++)for(let b=0;b<3;b++)A[a][b]+=J[a][i]*J[b][i];
 return {s,r,A,iterations:it}}

// Estimate the robot pose (x, y, heading) from detections; camera height, pitch and mount offsets are assumed known.
// minPixelSigma floors the per-corner noise used for the covariance, for systematic corner bias the residual cannot show.
function estimate(detections,{camera=defaultCamera(),tags=defaultTags(),maxIterations=30,huberPx=2,outlierPx=4,initial=null,minPixelSigma=0}={}){
 let cam={...defaultCamera(),...camera},K=intrinsics(cam),byId=new Map(tags.map(t=>[t.id,t]));
 let obs=detections.filter(d=>byId.has(d.id)).map(d=>({id:d.id,corners:d.corners,world:tagCorners(byId.get(d.id))})),dropped=[];
 if(!obs.length)return null;
 for(;;){let start=initial?[initial.x,initial.y,initial.headingDeg*DEG]:initialPose(obs,cam,K);if(!start)return null;
  let fit=refine(obs,cam,K,start,maxIterations,huberPx),perTag=obs.map((o,k)=>Math.sqrt(fit.r.slice(8*k,8*k+8).reduce((s,v)=>s+v*v,0)/4));
  let worst=perTag.indexOf(Math.max(...perTag));
  if(obs.length>=3&&perTag[worst]>outlierPx){dropped.push(obs[worst].id);obs.splice(worst,1);continue}
  let m=fit.r.length,sq=fit.r.reduce((s,v)=>s+v*v,0),cov=inverse3(fit.A),s2=Math.max(m>3?sq/(m-3):0,minPixelSigma**2);
  let covariance=cov&&cov.map(row=>row.map(v=>v*s2));
  return {x:fit.s[0],y:fit.s[1],headingDeg:fit.s[2]/DEG,rmsPx:Math.sqrt(sq/m),covariance,biasCovariance:cov&&cov.map(row=>row.map(v=>v*minPixelSigma**2)),
   std:covariance?{x:Math.sqrt(Math.max(covariance[0][0],0)),y:Math.sqrt(Math.max(covariance[1][1],0)),headingDeg:Math.sqrt(Math.max(covariance[2][2],0))/DEG}:null,
   tagIds:obs.map(o=>o.id),outlierTagIds:dropped,iterations:fit.iterations}}}

// Decides whether an estimate is trustworthy enough to act on. Motion blur over the exposure (from the gyro rate and
// drive speed) biases the corners in a way the reprojection residual does not reveal, and a single distant tag gives a
// weakly constrained pose, so both are rejected rather than fused.
function qualityGate(est,{camera=defaultCamera(),tags=defaultTags(),turnRateDegS=0,speed=0,exposureMs=8,maxBlurPx=2,singleTagMaxRange=1.5}={}){
 if(!est)return {ok:false,blurPx:0,reasons:['none']};let cam={...defaultCamera(),...camera},K=intrinsics(cam),T=exposureMs/1000,reasons=[];
 let blurPx=K.fx*Math.abs(turnRateDegS)*DEG*T+K.fx*Math.abs(speed)*T/1.5;if(blurPx>maxBlurPx)reasons.push('blur');
 if(est.tagIds.length<2){let tag=tags.find(t=>t.id===est.tagIds[0]),range=tag?Math.hypot(tag.x-est.x,tag.y-est.y):Infinity;if(range>singleTagMaxRange)reasons.push('singleFarTag')}
 return {ok:!reasons.length,blurPx,reasons}}

// Kalman filter on (x, y, heading) fusing vision estimates with optional robot-frame odometry.
function createPoseFilter({processStd={xy:.02,headingDeg:1},measurementFloor={xy:.002,headingDeg:.1}}={}){
 let s=null,P=null;const diag=(a,b,c)=>[[a,0,0],[0,b,0],[0,0,c]];
 return {
  get state(){return s&&{x:s[0],y:s[1],headingDeg:s[2]/DEG,covariance:P.map(row=>[...row]),std:{x:Math.sqrt(P[0][0]),y:Math.sqrt(P[1][1]),headingDeg:Math.sqrt(P[2][2])/DEG}}},
  reset(){s=null;P=null},
  predict(dt=0,odom={}){if(!s)return;let {dx=0,dy=0,dHeadingDeg=0}=odom,c=Math.cos(s[2]),n=Math.sin(s[2]);s=[s[0]+c*dx-n*dy,s[1]+n*dx+c*dy,wrap(s[2]+dHeadingDeg*DEG)];
   let q=processStd.xy**2*dt,qh=(processStd.headingDeg*DEG)**2*dt;P=P.map((row,i)=>row.map((v,j)=>v+(i===j?(i<2?q:qh):0)))},
  update(m){if(!m)return this.state;let z=[m.x,m.y,m.headingDeg*DEG],sd=m.std||{x:.05,y:.05,headingDeg:2};
   let floor=diag(measurementFloor.xy**2,measurementFloor.xy**2,(measurementFloor.headingDeg*DEG)**2);
   let R=m.covariance?m.covariance.map((row,i)=>row.map((v,j)=>v+floor[i][j])):diag(sd.x**2,sd.y**2,(sd.headingDeg*DEG)**2).map((row,i)=>row.map((v,j)=>v+floor[i][j]));
   if(!s){s=z;P=R;return this.state}
   let Sinv=inverse3(P.map((row,i)=>row.map((v,j)=>v+R[i][j])));if(!Sinv)return this.state;
   let Kg=P.map(row=>[0,1,2].map(j=>row.reduce((acc,v,k)=>acc+v*Sinv[k][j],0))),y=[z[0]-s[0],z[1]-s[1],wrap(z[2]-s[2])];
   s=s.map((v,i)=>v+Kg[i].reduce((acc,k,j)=>acc+k*y[j],0));s[2]=wrap(s[2]);
   P=P.map((row,i)=>row.map((v,j)=>v-[0,1,2].reduce((acc,k)=>acc+Kg[i][k]*P[k][j],0)));return this.state}}}

// Solves against each candidate layout (the HIVE pose combinations) and keeps the best: a wrong HIVE pose puts a
// cluster in the wrong place and orientation, so it either fits badly or loses that cluster's tags as outliers.
function estimateBest(detections,{layouts,...options}){let best=null;
 for(const layout of layouts){let e=estimate(detections,{...options,tags:layout.tags});if(!e)continue;
  if(!best||e.outlierTagIds.length<best.estimate.outlierTagIds.length||e.outlierTagIds.length===best.estimate.outlierTagIds.length&&e.rmsPx<best.estimate.rmsPx)best={estimate:e,layout}}
 // A HIVE none of whose tags were seen stays undetermined.
 if(best){let seen=new Set(best.layout.tags.filter(t=>best.estimate.tagIds.includes(t.id)).map(t=>t.hive));
  best={...best,layout:{...best.layout,hiveUp:Object.fromEntries(Object.entries(best.layout.hiveUp||{}).map(([k,v])=>[k,seen.has(k)?v:null]))}}}
 return best}

// Detect over several frames (independent pixel noise, robot static) and fuse the per-frame estimates. tags are where
// the tags really are; layouts are the candidate maps the solver may use, chosen on the first frame.
function localize({robot,camera,tags=defaultTags(),layouts=[{tags}],pixelNoise=.5,seed=1,frames=1,dropout=0,occluded=null}={}){
 let filter=createPoseFilter(),last=null,frameEstimates=[],chosen=null;
 for(let k=0;k<Math.max(1,frames|0);k++){last=detect({robot,camera,tags,pixelNoise,seed:seed+k*7919,dropout,occluded});
  let e;if(!chosen){let best=estimateBest(last.detections,{camera:last.camera,layouts});chosen=best&&best.layout;e=best&&best.estimate}else e=estimate(last.detections,{camera:last.camera,tags:chosen.tags});
  frameEstimates.push(e);if(e)filter.update(e)}
 let fused=filter.state,est=fused&&{...frameEstimates.filter(Boolean).pop(),x:fused.x,y:fused.y,headingDeg:fused.headingDeg,std:fused.std,covariance:fused.covariance};
 return {estimate:est,hiveUp:chosen&&chosen.hiveUp,detections:last.detections,rejected:last.rejected,camera:last.camera,cameraPose:last.cameraPose,framesUsed:frameEstimates.filter(Boolean).length,
  error:est?{x:est.x-robot.x,y:est.y-robot.y,position:Math.hypot(est.x-robot.x,est.y-robot.y),headingDeg:wrap((est.headingDeg-robot.headingDeg)*DEG)/DEG}:null}}

const api={TAG_SIZE,defaultCamera,defaultTags,hiveLayouts,cellFrame,tagFrame,estimateBest,intrinsics,tagCorners,cameraPose,project,toCamera,detect,estimate,localize,createPoseFilter,qualityGate};
if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.FiducialLocalizer=api;
})(typeof window!=='undefined'?window:globalThis);
