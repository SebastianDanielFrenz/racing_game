import json
import hashlib
from pathlib import Path
import struct
import tempfile
import unittest
from geometry import audit, export, load_glb

class GeometryTests(unittest.TestCase):
    def tetra(self):
        a,b,c,d=(0,0,0),(1,0,0),(0,1,0),(0,0,1)
        return [(a,c,b),(a,b,d),(a,d,c),(b,c,d)]
    def test_closed_tetra_is_not_cfd_certification(self):
        report=audit(self.tetra(),1e-6)
        self.assertTrue(report['closed_edge_topology'])
        self.assertFalse(report['cfd_ready'])
    def test_missing_face_is_detected(self):
        report=audit(self.tetra()[:-1],1e-6)
        self.assertEqual(report['boundary_edges'],3)
        self.assertEqual(len(report['boundary_components']),1)
        self.assertTrue(report['boundary_components'][0]['simple_closed_loop'])
        self.assertEqual(report['boundary_components'][0]['edges'],3)
    def test_reversed_face_is_detected(self):
        triangles=self.tetra();triangles[0]=triangles[0][::-1]
        self.assertEqual(audit(triangles,1e-6)['inconsistent_winding_edges'],3)
    def test_duplicate_degenerate_faces_are_detected(self):
        triangles=self.tetra();triangles += [triangles[0],((0,0,0),)*3]
        report=audit(triangles,1e-6)
        self.assertEqual(report['duplicate_faces'],1)
        self.assertEqual(report['degenerate_triangles'],1)
        self.assertGreater(report['nonmanifold_edges'],0)
    def test_hierarchy_strided_accessor_and_iso_export(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);binary=b''.join(struct.pack('<ffff',*p,99) for p in [(0,0,0),(1,0,0),(0,1,0)])
            doc={'asset':{'version':'2.0'},'buffers':[{'byteLength':len(binary)}],
                 'bufferViews':[{'buffer':0,'byteLength':len(binary),'byteStride':16}],
                 'accessors':[{'bufferView':0,'componentType':5126,'count':3,'type':'VEC3'}],
                 'materials':[{'name':'paint'}],
                 'meshes':[{'primitives':[{'attributes':{'POSITION':0},'material':0}]}],
                 'nodes':[{'name':'susp_FL','translation':[0,2,0],'children':[1]}, {'name':'body','mesh':0,'translation':[3,0,4]}],
                 'scenes':[{'nodes':[0]}],'scene':0}
            encoded=json.dumps(doc).encode();encoded+=b' '*((-len(encoded))%4)
            content=struct.pack('<II',len(encoded),0x4e4f534a)+encoded+struct.pack('<II',len(binary),0x004e4942)+binary
            source=root/'fixture.glb';source.write_bytes(struct.pack('<III',0x46546c67,2,12+len(content))+content)
            self.assertEqual(load_glb(source)[0][1][0][0],(4,3,2))
            report=export(source,root/'out')
            self.assertEqual(report['combined']['bounds_iso_m']['max'],[4,4,3])
            self.assertIn('v 4 3 2',(root/'out'/'hypercar_rest_iso.obj').read_text())
            vehicle=root/'vehicle.json';vehicle.write_text(json.dumps({'wheels':[{'name':'FL','attachment_local':[4,3,2]}]}))
            aligned=export(source,root/'aligned',vehicle=vehicle)
            self.assertEqual(aligned['body_alignment_iso_m'],[4,3,0])
            self.assertEqual(aligned['combined']['bounds_iso_m']['max'],[8,7,3])
            self.assertIn('chassis body origin',aligned['frame'])
            flow=root/'flow.json'
            selection={'format':'physics_sim.visual_flow_selection/1','source_glb_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                       'ranges':[{'node':'body','material':'paint','first_triangle':0,'triangle_count':1}]}
            flow.write_text(json.dumps(selection))
            self.assertEqual(load_glb(source,flow=flow),load_glb(source))
            selection['ranges'][0]['triangle_count']=2;flow.write_text(json.dumps(selection))
            with self.assertRaisesRegex(ValueError,'exceeds'):load_glb(source,flow=flow)
            selection['source_glb_sha256']='0'*64;flow.write_text(json.dumps(selection))
            with self.assertRaisesRegex(ValueError,'match'):load_glb(source,flow=flow)
            with self.assertRaises(ValueError):export(source,root/'bad',0)
            # Mirrored scene transforms must preserve outward face winding.
            doc['nodes'][0]['scale']=[-1,1,1]
            encoded=json.dumps(doc).encode();encoded+=b' '*((-len(encoded))%4)
            content=struct.pack('<II',len(encoded),0x4e4f534a)+encoded+struct.pack('<II',len(binary),0x004e4942)+binary
            source.write_bytes(struct.pack('<III',0x46546c67,2,12+len(content))+content)
            self.assertEqual(load_glb(source)[0][1][0],((4,-3,2),(4,-3,3),(4,-4,2)))

if __name__=='__main__':unittest.main()
