import cv2
import numpy as np
from skrobot.coordinates.math import quaternion_from_axis_angle


def rotation_vector_to_quaternion(rvec):
    """Convert rotation vector to quaternion.

    Parameters
    ----------
    rvec : list or numpy.ndarray
        vector of shape (3,)

    Returns
    -------
    q : numpy.ndarray
        quaternion
    """
    rvec = np.array(rvec).reshape(-1)
    theta = np.linalg.norm(rvec)
    if theta == 0:
        return np.array([1, 0, 0, 0], dtype=np.float64)
    axis = rvec / theta
    return quaternion_from_axis_angle(theta, axis)


def solve_pnp(
        canonical_points,
        projections,
        camera_K,
        method=cv2.SOLVEPNP_EPNP,
        refinement=True,
        dist_coeffs=np.array([])):

    n_canonial_points = len(canonical_points)
    n_projections = len(projections)

    if n_canonial_points != n_projections:
        raise ValueError(
            "Expected canonical_points and projections to "
            "have the same length, but they are length {} and {}."
            .format(n_canonial_points, n_projections))

    # Process points to remove any NaNs
    canonical_points_proc = []
    projections_proc = []
    for canon_pt, proj in zip(canonical_points, projections):
        if canon_pt is None \
                or len(canon_pt) == 0 \
                or canon_pt[0] is None \
                or canon_pt[1] is None \
                or proj is None \
                or len(proj) == 0 \
                or proj[0] is None \
                or proj[1] is None:
            continue

        canonical_points_proc.append(canon_pt)
        projections_proc.append(proj)

    if len(canonical_points_proc) == 0:
        return False, None, None

    canonical_points_proc = np.array(canonical_points_proc)
    projections_proc = np.array(projections_proc)

    try:
        inlier_thresh_px = 5.0
        pnp_retval, rvec, tvec, indices = cv2.solvePnPRansac(
            canonical_points_proc.reshape(
                canonical_points_proc.shape[0], 1, 3),
            projections_proc.reshape(projections_proc.shape[0], 1, 2),
            camera_K,
            dist_coeffs,
            flags=method,
            reprojectionError=inlier_thresh_px,
        )

        if refinement:
            canonical_points_proc = canonical_points_proc[indices.reshape(-1)]
            projections_proc = projections_proc[indices.reshape(-1)]
            pnp_retval, rvec, tvec = cv2.solvePnP(
                canonical_points_proc.reshape(
                    canonical_points_proc.shape[0], 1, 3),
                projections_proc.reshape(projections_proc.shape[0], 1, 2),
                camera_K,
                dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
                useExtrinsicGuess=True,
                rvec=rvec,
                tvec=tvec,
            )

        translation = tvec[:, 0]
        quaternion = rotation_vector_to_quaternion(rvec.reshape(-1))
    except Exception as e:
        print(e)
        pnp_retval = False
        quaternion = None
        translation = None

    return pnp_retval, quaternion, translation
