import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { ArmState, Side } from './types';
import { SIDES } from './types';
import type { DeployIK } from './wasm';

interface ArmMeshes { joints: THREE.Mesh[]; links: THREE.Mesh[]; axes: THREE.ArrowHelper[]; tool: THREE.AxesHelper }

export class ArmRenderer {
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(42, 1, 1, 4000);
  private readonly renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  private readonly arms = {} as Record<Side, ArmMeshes>;
  private readonly resizeObserver: ResizeObserver;

  constructor(private readonly host: HTMLElement, private readonly ik: DeployIK) {
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    host.appendChild(this.renderer.domElement);
    this.scene.background = new THREE.Color(0x07101d);
    this.scene.fog = new THREE.Fog(0x07101d, 800, 1700);
    this.camera.position.set(720, -760, 520);
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(160, 0, 160);
    const controls = new OrbitControls(this.camera, this.renderer.domElement);
    controls.target.set(160, 0, 170);
    controls.enableDamping = true;
    controls.addEventListener('change', () => this.render());
    this.scene.add(new THREE.HemisphereLight(0xb9dcff, 0x18202e, 2.1));
    const key = new THREE.DirectionalLight(0xffffff, 2.8);
    key.position.set(350, -300, 700); key.castShadow = true; this.scene.add(key);
    const grid = new THREE.GridHelper(1000, 20, 0x29415f, 0x172940);
    grid.rotateX(Math.PI / 2); this.scene.add(grid);
    const base = new THREE.Mesh(new THREE.BoxGeometry(90, 300, 45), new THREE.MeshStandardMaterial({ color: 0x1d3048, metalness: .55, roughness: .35 }));
    base.position.set(0, 0, -25); base.castShadow = true; this.scene.add(base);
    for (const side of SIDES) this.arms[side] = this.makeArm(side);
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(host);
    const loop = () => { controls.update(); this.renderer.render(this.scene, this.camera); requestAnimationFrame(loop); };
    loop();
  }

  private makeArm(side: Side): ArmMeshes {
    const group = new THREE.Group(); this.scene.add(group);
    const color = side === 'left' ? 0x3296ff : 0xff8051;
    const jointMaterial = new THREE.MeshStandardMaterial({ color, metalness: .65, roughness: .28 });
    const linkMaterial = new THREE.MeshStandardMaterial({ color: side === 'left' ? 0x91c9ff : 0xffb99e, metalness: .4, roughness: .38 });
    const joints = Array.from({ length: 7 }, () => {
      const mesh = new THREE.Mesh(new THREE.SphereGeometry(15, 20, 14), jointMaterial); mesh.castShadow = true; group.add(mesh); return mesh;
    });
    const links = Array.from({ length: 7 }, () => {
      const mesh = new THREE.Mesh(new THREE.CylinderGeometry(9, 9, 1, 16), linkMaterial); mesh.castShadow = true; group.add(mesh); return mesh;
    });
    const axes = Array.from({ length: 7 }, () => {
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), undefined, 42, 0xf7d04b, 10, 5); group.add(arrow); return arrow;
    });
    const tool = new THREE.AxesHelper(55); group.add(tool);
    return { joints, links, axes, tool };
  }

  update(state: ArmState): void {
    for (const side of SIDES) {
      const chain = this.ik.chain(side, state[side]);
      const meshes = this.arms[side];
      for (let i = 0; i < 7; i++) {
        const a = new THREE.Vector3(...chain.positions[i] as [number, number, number]);
        const b = new THREE.Vector3(...chain.positions[i + 1] as [number, number, number]);
        meshes.joints[i].position.copy(a);
        meshes.axes[i].position.copy(a);
        meshes.axes[i].setDirection(new THREE.Vector3(...chain.axes[i] as [number, number, number]).normalize());
        this.placeCylinder(meshes.links[i], a, b);
      }
      const fk = this.ik.fk(side, state[side]);
      meshes.tool.position.set(...fk.position as [number, number, number]);
      const m = new THREE.Matrix4().set(
        fk.rotation[0], fk.rotation[1], fk.rotation[2], 0,
        fk.rotation[3], fk.rotation[4], fk.rotation[5], 0,
        fk.rotation[6], fk.rotation[7], fk.rotation[8], 0,
        0, 0, 0, 1,
      );
      meshes.tool.quaternion.setFromRotationMatrix(m);
    }
  }

  private placeCylinder(mesh: THREE.Mesh, a: THREE.Vector3, b: THREE.Vector3): void {
    const delta = b.clone().sub(a); const length = delta.length();
    mesh.visible = length > .01;
    if (!mesh.visible) return;
    mesh.position.copy(a).add(b).multiplyScalar(.5);
    mesh.scale.set(1, length, 1);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize());
  }

  private resize(): void {
    const width = this.host.clientWidth, height = this.host.clientHeight;
    this.renderer.setSize(width, height, false); this.camera.aspect = width / Math.max(height, 1); this.camera.updateProjectionMatrix(); this.render();
  }

  private render(): void { this.renderer.render(this.scene, this.camera); }
}
