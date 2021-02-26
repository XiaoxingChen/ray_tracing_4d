# Ray Tracing in 4-dimensional World

<p>
This repo extends a simplest 3-dimensional ray tracing engine(CPU implementation) to 4-dimensional world.
</p>


<table>
<tr>
<th>.</th><th>3D</th><th>4D</th>
</tr>
<tr>
<td>Parameterization of Rotation</td>
<td>SO3, axis-angle or quaternion</td>
<td>SO4, bi-vector or two quaternions
(Rodrigous formula doesn't hold anymore)
</td>
</tr>
<tr>
<td>Camera Model</td>
<td>
<img src="https://render.githubusercontent.com/render/math?math=\left[\begin{matrix}f_x%260%26c_x\\0%26f_y%26c_y\\0%260%261\end{matrix}\right]">
</td>
<td>
<img src="https://render.githubusercontent.com/render/math?math=\left[\begin{matrix}f_x%260%260%26c_x\\0%26f_y%260%26c_y\\0%260%26f_z%26c_z\\0%260%260%261\end{matrix}\right]">
</td>
</tr>
<tr>
<td>Image of a 3D object</td>
<td>
<img src="https://user-images.githubusercontent.com/16934019/109266421-0495cc00-7843-11eb-80d5-ec138e513b9a.png">
<br>by 3D camera, resolution: 640x480
</td>
<td>
<img src="https://user-images.githubusercontent.com/16934019/109257943-3a7f8400-7834-11eb-8431-3572d9114378.gif">
<br>by 4D camera, resolution: 640x480x32
</td>
</tr>
</table>

<h2>References</h2>
<ol>
    <li> <a href="https://miegakure.com/">Miegakure</a> </li>
    <li> <a href="https://github.com/pygae/clifford">python geometric algebra library</a></li>
    <li> <a href="https://www.scratchapixel.com/">scratch a pixel</a> </li>
    <li> <a href="https://raytracing.github.io/">ray tracing in one weekend</a></li>
    <li> <a href="https://hal.archives-ouvertes.fr/hal-00713697/document">decomposition of n-d rotations</a></li>
    <li> <a href="https://planetmath.org/decompositionoforthogonaloperatorsasrotationsandreflections">block diagonalize rotation matrix</a></li>
</ol>
