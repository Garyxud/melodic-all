/**
\page dg_tutorial_inverted_pendulum_python Python module

\section dg_tutorial_inverted_pendulum_python_intro Introduction

Generating python bindings for new Entity classes is straightforward. We only need to add the following lines into file
<c>src/CMakeLists.txt</c>:

<code>
DYNAMIC_GRAPH_PYTHON_MODULE("tutorial" ${LIBRARY_NAME} wrap)
</code>

This will create and install a python module called <c>dynamic_graph.tutorial</c>, linked with library
<c>${LIBRARY_NAME} (libdynamic-graph-tutorial.so)</c>. When importing this module, two new python classes deriving from
<c>Entity</c> are created: \li InvertedPendulum, and \li FeedbackController. \sa <c>src/simu.py</c> for an example
python script.
 */
