## BUGs in current system:
- (Finish)Sometimes system stucked in tsdfFusion(multi-thread), solution: try to change the in_tsdffusion as the atomic operation
- Pose tracking may failed(Hardly happen)
- (Finish)System may crush during the get full meshes(multi-thread)
- (Finish)Segmentation visualization may be influenced by Mesh simplification(multi-thread), scattered. 
- (? should be finish)visualization: crushed(server, hardly happen).
- /home/wlsdzyzl/learning/CollaborativeFusionWithSemantic/src/collaborative_fusion/src/scn_cpp/sparseconvnet/SCN/CUDA/SubmanifoldRules_cuda.cu:213: void _dGenerateChuckRulebook(int *, int *, int *, int *, int *, int *, short *, int *, int, int, int, int, int, int): block: [8,0,0], thread: [34,0,0] Assertion `idx != -1` failed.


- Room Detection somtimes the distance_matrix is wrong

- Slow fresh rate


### when we add submap integration
- mesh simplification
- chunk integration
- tofloat16
