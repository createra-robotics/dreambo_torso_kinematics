//! Micro-benchmarks for arm + neck kinematics. Target: sub-µs FK.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use dreambo_torso_kinematics::arm::{
    forward_kinematics, inverse_kinematics, SphericalLinkGeometry,
};
use dreambo_torso_kinematics::neck::DreamboNeckKinematics;

fn bench_arm_fk(c: &mut Criterion) {
    let geom = SphericalLinkGeometry::default_left();
    c.bench_function("arm_fk", |b| {
        b.iter(|| {
            let _ = forward_kinematics(black_box(&geom), black_box(0.1), black_box(0.1)).unwrap();
        });
    });
}

fn bench_arm_ik(c: &mut Criterion) {
    let geom = SphericalLinkGeometry::default_left();
    let r = forward_kinematics(&geom, 0.1, 0.1).unwrap().r_world_ee;
    c.bench_function("arm_ik", |b| {
        b.iter(|| {
            let _ = inverse_kinematics(black_box(&geom), black_box(&r)).unwrap();
        });
    });
}

fn bench_neck_fk(c: &mut Criterion) {
    let neck = DreamboNeckKinematics::new();
    c.bench_function("neck_fk", |b| {
        b.iter(|| {
            let _ = neck.fk(black_box([0.1, 0.2, 0.0]));
        });
    });
}

criterion_group!(benches, bench_arm_fk, bench_arm_ik, bench_neck_fk);
criterion_main!(benches);
