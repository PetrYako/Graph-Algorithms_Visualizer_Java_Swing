package visualizer;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.*;
import java.util.List;
import java.util.concurrent.ExecutionException;

public class MainFrame extends JFrame {
    private JPanel graphPanel;
    private JLabel modeLabel;
    private JLabel algorithmInstructionLabel;
    private String currentMode;
    private String algorithm;
    private VertexPanel firstSelectedVertex = null;
    private List<Edge> edges = new ArrayList<>();

    private static final String ADD_VERTEX = "Add a Vertex";
    private static final String ADD_EDGE = "Add an Edge";
    private static final String REMOVE_VERTEX = "Remove a Vertex";
    private static final String REMOVE_EDGE = "Remove an Edge";
    private static final String NONE = "None";
    private static final String DFS = "Depth-First Search";
    private static final String BFS = "Breadth-First Search";
    private static final String DIJKSTRA = "Dijkstra's Algorithm";
    private static final String PRIMS = "Prim's Algorithm"; // Added

    public MainFrame() {
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(800, 600);
        setTitle("Graph-Algorithms Visualizer");
        setLocationRelativeTo(null);

        JMenuBar menuBar = createMenuBar();
        setJMenuBar(menuBar);

        modeLabel = new JLabel();
        modeLabel.setForeground(Color.WHITE);
        modeLabel.setName("Mode");
        modeLabel.setBounds(600, 10, 180, 20);
        add(modeLabel);
        setMode("Add a Vertex");

        graphPanel = new GraphPanel();
        algorithmInstructionLabel = new JLabel();
        algorithmInstructionLabel.setName("Display");
        algorithmInstructionLabel.setBackground(Color.WHITE);
        algorithmInstructionLabel.setHorizontalAlignment(SwingConstants.CENTER);
        add(graphPanel);
        add(algorithmInstructionLabel, BorderLayout.SOUTH);

        setVisible(true);
    }

    private JMenuBar createMenuBar() {
        JMenuBar menuBar = new JMenuBar();

        JMenu fileMenu = new JMenu("File");
        menuBar.add(fileMenu);
        fileMenu.add(createMenuItem("New", e -> resetGraph()));
        fileMenu.add(createMenuItem("Exit", e -> System.exit(0)));

        JMenu modeMenu = new JMenu("Mode");
        menuBar.add(modeMenu);
        modeMenu.add(createMenuItem(ADD_VERTEX, e -> setMode(ADD_VERTEX)));
        modeMenu.add(createMenuItem(ADD_EDGE, e -> setMode(ADD_EDGE)));
        modeMenu.add(createMenuItem(REMOVE_VERTEX, e -> setMode(REMOVE_VERTEX)));
        modeMenu.add(createMenuItem(REMOVE_EDGE, e -> setMode(REMOVE_EDGE)));
        modeMenu.add(createMenuItem(NONE, e -> setMode(NONE)));

        JMenu algorithmsMenu = new JMenu("Algorithms");
        menuBar.add(algorithmsMenu);
        algorithmsMenu.add(createMenuItem(DFS, e -> {
            algorithmInstructionLabel.setText("Please choose a starting vertex");
            setMode(NONE);
            algorithm = DFS;
        }));
        algorithmsMenu.add(createMenuItem(BFS, e -> {
            algorithmInstructionLabel.setText("Please choose a starting vertex");
            setMode(NONE);
            algorithm = BFS;
        }));
        algorithmsMenu.add(createMenuItem(DIJKSTRA, e -> {
            algorithmInstructionLabel.setText("Please choose a starting vertex");
            setMode(NONE);
            algorithm = DIJKSTRA;
        }));
        algorithmsMenu.add(createMenuItem(PRIMS, e -> { // Added
            algorithmInstructionLabel.setText("Please choose a starting vertex");
            setMode(NONE);
            algorithm = PRIMS;
        }));

        return menuBar;
    }

    private JMenuItem createMenuItem(String name, ActionListener actionListener) {
        JMenuItem menuItem = new JMenuItem(name);
        menuItem.setName(name);
        if (actionListener != null) {
            menuItem.addActionListener(actionListener);
        }
        return menuItem;
    }

    private void resetGraph() {
        graphPanel.removeAll();
        graphPanel.revalidate();
        graphPanel.repaint();
        edges.clear();
        algorithmInstructionLabel.setText("");
        algorithmInstructionLabel.repaint();
        algorithm = null;
        setMode(ADD_VERTEX);
    }

    private void setMode(String mode) {
        currentMode = mode;
        modeLabel.setText("Current Mode -> " + currentMode);
        firstSelectedVertex = null;
    }

    private void handleAlgorithm(MouseEvent e) {
        Component clickedComponent = SwingUtilities.getDeepestComponentAt(graphPanel, e.getX(), e.getY());
        if (clickedComponent instanceof VertexPanel || (clickedComponent.getParent() instanceof VertexPanel)) {
            VertexPanel selectedVertex = (clickedComponent instanceof VertexPanel) ? (VertexPanel) clickedComponent : (VertexPanel) clickedComponent.getParent();
            algorithmInstructionLabel.setText("Please wait...");
            new AlgorithmWorker(selectedVertex).execute();
        }
    }

    class GraphPanel extends JPanel {
        private List<VertexPanel> vertices = new ArrayList<>();

        public GraphPanel() {
            setLayout(null);
            setBackground(Color.BLACK);
            setName("Graph");

            addMouseListener(new MouseAdapter() {
                @Override
                public void mouseClicked(MouseEvent e) {
                    handleMouseClick(e);
                }
            });

            setVisible(true);
        }

        private void handleMouseClick(MouseEvent e) {
            if (currentMode.equals(ADD_VERTEX)) {
                handleAddVertex(e);
            } else if (currentMode.equals(ADD_EDGE)) {
                handleAddEdge(e);
            } else if (currentMode.equals(REMOVE_VERTEX)) {
                handleRemoveVertex(e);
            } else if (currentMode.equals(REMOVE_EDGE)) {
                handleRemoveEdge(e);
            } else if (currentMode.equals(NONE) && algorithm != null) {
                handleAlgorithm(e);
            } else {
                firstSelectedVertex = null;
            }
        }

        private void handleRemoveEdge(MouseEvent e) {
            Component clickedComponent = SwingUtilities.getDeepestComponentAt(GraphPanel.this, e.getX(), e.getY());
            if (clickedComponent instanceof VertexPanel || (clickedComponent.getParent() instanceof VertexPanel)) {
                VertexPanel selectedVertex = (clickedComponent instanceof VertexPanel) ? (VertexPanel) clickedComponent : (VertexPanel) clickedComponent.getParent();
                removeVertexEdges(selectedVertex);
                repaint();
                revalidate();
            } else if (clickedComponent instanceof Edge selectedEdge) {
                removeEdge(selectedEdge);
                repaint();
                revalidate();
            }
        }

        private void handleRemoveVertex(MouseEvent e) {
            Component clickedComponent = SwingUtilities.getDeepestComponentAt(GraphPanel.this, e.getX(), e.getY());
            if (clickedComponent instanceof VertexPanel || (clickedComponent.getParent() instanceof VertexPanel)) {
                VertexPanel selectedVertex = (clickedComponent instanceof VertexPanel) ? (VertexPanel) clickedComponent : (VertexPanel) clickedComponent.getParent();
                removeVertex(selectedVertex);
                repaint();
                revalidate();
            }
        }

        private void handleAddEdge(MouseEvent e) {
            Component clickedComponent = SwingUtilities.getDeepestComponentAt(GraphPanel.this, e.getX(), e.getY());
            if (clickedComponent instanceof VertexPanel || (clickedComponent.getParent() instanceof VertexPanel)) {
                VertexPanel selectedVertex = (clickedComponent instanceof VertexPanel) ? (VertexPanel) clickedComponent : (VertexPanel) clickedComponent.getParent();
                if (firstSelectedVertex == null) {
                    firstSelectedVertex = selectedVertex;
                } else if (!firstSelectedVertex.equals(selectedVertex)) {
                    String edgeWeight = null;
                    boolean validInput = false;
                    while (!validInput) {
                        JTextField textField = new JTextField(5);
                        Object[] message = {"Enter Weight", textField};
                        int option = JOptionPane.showConfirmDialog(null, message, "Edge Weight", JOptionPane.OK_CANCEL_OPTION);
                        if (option == JOptionPane.OK_OPTION) {
                            edgeWeight = textField.getText();
                            try {
                                int weight = Integer.parseInt(edgeWeight);
                                validInput = true;

                                List<Edge> existingEdge = findEdges(firstSelectedVertex, selectedVertex);
                                if (!existingEdge.isEmpty()) {
                                    existingEdge.forEach(ee -> {
                                        remove(ee);
                                        if (ee.weightLabel != null) {
                                            remove(ee.weightLabel);
                                        }
                                        edges.remove(ee);
                                    });
                                }

                                addEdge(firstSelectedVertex, selectedVertex, weight);
                                repaint();
                                revalidate();
                            } catch (NumberFormatException ex) {
                                textField.setText("");
                            }
                        } else {
                            return;
                        }
                    }
                    firstSelectedVertex = null;
                }
            }
        }

        private void handleAddVertex(MouseEvent e) {
            firstSelectedVertex = null;
            String vertexId = null;
            boolean validInput = false;

            for (VertexPanel vertex : vertices) {
                if (vertex.getBounds().contains(e.getPoint())) {
                    return;
                }
            }

            while (!validInput) {
                JTextField textField = new JTextField(5);
                Object[] message = {"Enter Vertex ID (Should be 1 char):", textField};
                int option = JOptionPane.showConfirmDialog(null, message, "Vertex", JOptionPane.OK_CANCEL_OPTION);
                if (option == JOptionPane.OK_OPTION) {
                    vertexId = textField.getText();
                    if (vertexId != null && vertexId.length() == 1 && Character.isLetterOrDigit(vertexId.charAt(0))) {
                        validInput = true;
                    } else {
                        textField.setText("");
                    }
                } else {
                    return;
                }
            }

            VertexPanel vertex = new VertexPanel(vertexId);
            vertex.setBounds(e.getX() - 25, e.getY() - 25, 50, 50);
            add(vertex);
            vertices.add(vertex);
            repaint();
            revalidate();
        }

        private void addEdge(VertexPanel firstSelectedVertex, VertexPanel selectedVertex, int weight) {
            Edge edge = new Edge(firstSelectedVertex, selectedVertex, weight);
            add(edge);
            edges.add(edge);

            Edge edgeReverse = new Edge(selectedVertex, firstSelectedVertex, weight);
            add(edgeReverse);
            edges.add(edgeReverse);

            JLabel weightLabel = new JLabel(String.valueOf(edge.weight));
            weightLabel.setForeground(Color.WHITE);
            weightLabel.setName(String.format("EdgeLabel <%s -> %s>", edge.source.getVertexId(), edge.destination.getVertexId()));
            add(weightLabel);
            edge.weightLabel = weightLabel;
            edge.updatePosition(weightLabel);
        }

        private List<Edge> findEdges(VertexPanel source, VertexPanel destination) {
            List<Edge> edges = new ArrayList<>();
            for (Edge edge : MainFrame.this.edges) {
                if ((edge.source.equals(source) && edge.destination.equals(destination)) ||
                        (edge.source.equals(destination) && edge.destination.equals(source))) {
                    edges.add(edge);
                }
            }
            return edges;
        }

        private void removeVertex(VertexPanel vertex) {
            List<Edge> edgesToRemove = new ArrayList<>();
            for (Edge edge : edges) {
                if (edge.source.equals(vertex) || edge.destination.equals(vertex)) {
                    edgesToRemove.add(edge);
                }
            }
            edgesToRemove.forEach(edge -> {
                remove(edge);
                if (edge.weightLabel != null) {
                    remove(edge.weightLabel);
                }
                edges.remove(edge);
            });
            vertices.remove(vertex);
            remove(vertex);
        }

        private void removeEdge(Edge edge) {
            List<Edge> edgesToRemove = findEdges(edge.source, edge.destination);
            edgesToRemove.forEach(e -> {
                remove(e);
                if (e.weightLabel != null) {
                    remove(e.weightLabel);
                }
                edges.remove(e);
            });
        }

        private void removeVertexEdges(VertexPanel vertex) {
            List<Edge> edgesToRemove = new ArrayList<>();
            for (Edge edge : edges) {
                if (edge.source.equals(vertex) || edge.destination.equals(vertex)) {
                    edgesToRemove.add(edge);
                }
            }
            edgesToRemove.forEach(edge -> {
                remove(edge);
                if (edge.weightLabel != null) {
                    remove(edge.weightLabel);
                }
                edges.remove(edge);
            });
        }
    }

    class VertexPanel extends JPanel {
        private String vertexId;
        private Color color = Color.WHITE;

        public VertexPanel(String vertexId) {
            setLayout(new GridBagLayout());
            this.vertexId = vertexId;
            setName("Vertex " + vertexId);
            setOpaque(false);

            JLabel vertexIdLabel = new JLabel(vertexId);
            vertexIdLabel.setName("VertexLabel " + vertexId);
            vertexIdLabel.setForeground(Color.BLACK);
            add(vertexIdLabel);
        }

        @Override
        protected void paintComponent(Graphics g) {
            g.setColor(color);
            g.fillOval(10, 10, 30, 30);
        }

        public String getVertexId() {
            return vertexId;
        }

        public void setColor(Color color) {
            this.color = color;
            repaint();
        }
    }

    class Edge extends JComponent {
        private VertexPanel source;
        private VertexPanel destination;
        private JLabel weightLabel;
        private int weight;
        private Color color = Color.WHITE;

        public Edge(VertexPanel source, VertexPanel destination, int weight) {
            this.source = source;
            this.destination = destination;
            this.weight = weight;
            setLayout(null);
            setOpaque(false);
            setName(String.format("Edge <%s -> %s>", source.getVertexId(), destination.getVertexId()));

            setBounds(0, 0, 800, 600);
        }

        public void setColor(Color color) {
            this.color = color;
            repaint();
        }

        public void updatePosition(JLabel weightLabel) {
            Point sourcePoint = source.getLocation();
            Point destPoint = destination.getLocation();
            int sourceX = sourcePoint.x + source.getWidth() / 2;
            int sourceY = sourcePoint.y + source.getHeight() / 2;
            int destX = destPoint.x + destination.getWidth() / 2;
            int destY = destPoint.y + destination.getHeight() / 2;

            int weightLabelX = (sourceX + destX) / 2;
            int weightLabelY = ((sourceY + destY) / 2) + 10;
            weightLabel.setBounds(weightLabelX - 10, weightLabelY - 10, 30, 20);
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D) g;
            g2.setColor(color);
            g2.setStroke(new BasicStroke(2));

            Point sourcePoint = source.getLocation();
            Point destPoint = destination.getLocation();
            int sourceX = sourcePoint.x + source.getWidth() / 2;
            int sourceY = sourcePoint.y + source.getHeight() / 2;
            int destX = destPoint.x + destination.getWidth() / 2;
            int destY = destPoint.y + destination.getHeight() / 2;

            g2.drawLine(sourceX, sourceY, destX, destY);
        }
    }

    private class AlgorithmWorker extends SwingWorker<Void, String> {
        private VertexPanel startVertex;
        private List<String> traversalOrder = new ArrayList<>();
        private Map<VertexPanel, List<VertexPanel>> adjacencyList = new HashMap<>();
        private Map<VertexPanel, Integer> distances = new HashMap<>();
        private Map<VertexPanel, VertexPanel> parentMap = new HashMap<>();

        public AlgorithmWorker(VertexPanel startVertex) {
            this.startVertex = startVertex;
            buildAdjacencyList();
        }

        private void buildAdjacencyList() {
            GraphPanel gp = (GraphPanel) graphPanel;
            for (VertexPanel vertex : gp.vertices) {
                adjacencyList.put(vertex, new ArrayList<>());
            }
            for (Edge edge : edges) {
                // Avoid adding reverse edges twice in adjacency list
                if (!adjacencyList.get(edge.source).contains(edge.destination)) {
                    adjacencyList.get(edge.source).add(edge.destination);
                }
            }
        }

        @Override
        protected Void doInBackground() throws Exception {
            if (DFS.equals(algorithm)) {
                dfs(startVertex, new HashSet<>());
            } else if (BFS.equals(algorithm)) {
                bfs(startVertex);
            } else if (DIJKSTRA.equals(algorithm)) {
                dijkstra(startVertex);
            } else if (PRIMS.equals(algorithm)) {
                prims(startVertex);
            }
            return null;
        }

        private void dfs(VertexPanel vertex, Set<VertexPanel> visited) throws InterruptedException {
            if (isCancelled()) return;
            visited.add(vertex);
            traversalOrder.add(vertex.getVertexId());
            publish("Visit " + vertex.getVertexId());
            // Color the vertex as visited
            SwingUtilities.invokeLater(() -> vertex.setColor(Color.YELLOW));
            Thread.sleep(700); // Delay for visualization

            for (VertexPanel neighbor : adjacencyList.get(vertex)) {
                if (!visited.contains(neighbor)) {
                    // Find the edge between vertex and neighbor
                    Edge edge = findEdge(vertex, neighbor);
                    if (edge != null) {
                        // Color the edge
                        SwingUtilities.invokeLater(() -> edge.setColor(Color.YELLOW));
                        Thread.sleep(500);
                    }
                    dfs(neighbor, visited);
                }
            }
        }

        private void bfs(VertexPanel start) throws InterruptedException {
            Set<VertexPanel> visited = new HashSet<>();
            Queue<VertexPanel> queue = new LinkedList<>();
            queue.add(start);
            visited.add(start);
            traversalOrder.add(start.getVertexId());
            publish("Visit " + start.getVertexId());
            SwingUtilities.invokeLater(() -> start.setColor(Color.YELLOW));
            Thread.sleep(700);

            while (!queue.isEmpty()) {
                VertexPanel vertex = queue.poll();
                for (VertexPanel neighbor : adjacencyList.get(vertex)) {
                    if (!visited.contains(neighbor)) {
                        visited.add(neighbor);
                        queue.add(neighbor);
                        traversalOrder.add(neighbor.getVertexId());
                        // Find the edge between vertex and neighbor
                        Edge edge = findEdge(vertex, neighbor);
                        if (edge != null) {
                            // Color the edge
                            SwingUtilities.invokeLater(() -> edge.setColor(Color.YELLOW));
                            Thread.sleep(500);
                        }
                        publish("Visit " + neighbor.getVertexId());
                        SwingUtilities.invokeLater(() -> neighbor.setColor(Color.YELLOW));
                        Thread.sleep(700);
                    }
                }
            }
        }

        private void dijkstra(VertexPanel start) throws InterruptedException {
            GraphPanel gp = (GraphPanel) graphPanel;
            for (VertexPanel vertex : gp.vertices) {
                distances.put(vertex, Integer.MAX_VALUE);
            }
            distances.put(start, 0);
            SwingUtilities.invokeLater(() -> start.setColor(Color.RED));

            // Priority queue based on distance
            PriorityQueue<VertexDistancePair> pq = new PriorityQueue<>(Comparator.comparingInt(pair -> pair.distance));
            pq.add(new VertexDistancePair(start, 0));

            Set<VertexPanel> visited = new HashSet<>();

            while (!pq.isEmpty()) {
                VertexDistancePair currentPair = pq.poll();
                VertexPanel currentVertex = currentPair.vertex;
                int currentDistance = currentPair.distance;

                if (visited.contains(currentVertex)) {
                    continue;
                }

                visited.add(currentVertex);
                if (!currentVertex.equals(start)) { // Exclude starting vertex from display
                    traversalOrder.add(currentVertex.getVertexId() + "=" + currentDistance);
                    publish("Visit " + currentVertex.getVertexId() + " with cost " + currentDistance);
                }

                for (VertexPanel neighbor : adjacencyList.get(currentVertex)) {
                    if (visited.contains(neighbor)) {
                        continue;
                    }
                    // Find the edge between currentVertex and neighbor
                    Edge edge = findEdge(currentVertex, neighbor);
                    if (edge != null) {
                        int newDist = currentDistance + edge.weight;
                        if (newDist < distances.get(neighbor)) {
                            distances.put(neighbor, newDist);
                            pq.add(new VertexDistancePair(neighbor, newDist));
                        }
                        Thread.sleep(500);
                    }
                }
            }
        }

        private void prims(VertexPanel start) throws InterruptedException {
            // Initialize all vertices as not part of MST
            Set<VertexPanel> inMST = new HashSet<>();
            PriorityQueue<EdgeWeightPair> pq = new PriorityQueue<>(Comparator.comparingInt(pair -> pair.weight));
            Map<VertexPanel, VertexPanel> mstParents = new HashMap<>(); // Child -> Parent

            // Start with the starting vertex
            inMST.add(start);

            // Add all edges from the start vertex to the priority queue
            for (VertexPanel neighbor : adjacencyList.get(start)) {
                Edge edge = findEdge(start, neighbor);
                if (edge != null) {
                    pq.add(new EdgeWeightPair(edge, neighbor, edge.weight));
                }
            }

            while (!pq.isEmpty()) {
                EdgeWeightPair currentPair = pq.poll();
                Edge currentEdge = currentPair.edge;
                VertexPanel toVertex = currentPair.toVertex;

                if (inMST.contains(toVertex)) {
                    continue;
                }

                // Add toVertex to MST
                inMST.add(toVertex);
                mstParents.put(toVertex, currentEdge.source); // Set parent

                // Color the edge as part of MST
                SwingUtilities.invokeLater(() -> {
                    currentEdge.setColor(Color.YELLOW);
                    // Find the reverse edge and color it as well
                    Edge reverseEdge = findReverseEdge(currentEdge);
                    if (reverseEdge != null) {
                        reverseEdge.setColor(Color.YELLOW);
                    }
                });
                publish("Add " + toVertex.getVertexId() + "=" + currentEdge.source.getVertexId());
                Thread.sleep(700);

                // Add all edges from the newly added vertex to the priority queue
                for (VertexPanel neighbor : adjacencyList.get(toVertex)) {
                    if (!inMST.contains(neighbor)) {
                        Edge edge = findEdge(toVertex, neighbor);
                        if (edge != null) {
                            pq.add(new EdgeWeightPair(edge, neighbor, edge.weight));
                        }
                    }
                }
            }

            // Collect the parent-child pairs
            for (Map.Entry<VertexPanel, VertexPanel> entry : mstParents.entrySet()) {
                VertexPanel child = entry.getKey();
                VertexPanel parent = entry.getValue();
                traversalOrder.add(child.getVertexId() + "=" + parent.getVertexId());
            }
        }

        private Edge findEdge(VertexPanel source, VertexPanel destination) {
            for (Edge edge : edges) {
                if (edge.source.equals(source) && edge.destination.equals(destination)) {
                    return edge;
                }
            }
            return null;
        }

        private Edge findReverseEdge(Edge edge) {
            for (Edge e : edges) {
                if (e.source.equals(edge.destination) && e.destination.equals(edge.source) && e.weight == edge.weight) {
                    return e;
                }
            }
            return null;
        }

        @Override
        protected void done() {
            try {
                get();
                if (PRIMS.equals(algorithm)) {
                    // Generate the list of <Child>=<Parent> pairs
                    List<String> sortedPairs = new ArrayList<>(traversalOrder);
                    sortedPairs.sort((o1, o2) -> {
                        String child1 = o1.split("=")[0];
                        String child2 = o2.split("=")[0];
                        return child1.compareTo(child2);
                    });

                    StringBuilder sb = new StringBuilder();
                    sortedPairs.forEach(pair -> sb.append(pair).append(", "));
                    if (sb.length() >= 2) {
                        sb.setLength(sb.length() - 2);
                    }
                    algorithmInstructionLabel.setText(sb.toString());
                } else if (DIJKSTRA.equals(algorithm)) {
                    StringBuilder sb = new StringBuilder();
                    traversalOrder.sort(Comparator.naturalOrder());
                    traversalOrder.forEach(pair -> sb.append(pair).append(", "));
                    if (sb.length() >= 2) {
                        sb.setLength(sb.length() - 2);
                    }
                    algorithmInstructionLabel.setText(sb.toString());
                } else {
                    String algo = "";
                    if (algorithm.equals(DFS)) {
                        algo = "DFS";
                    } else {
                        algo = "BFS";
                    }
                    algorithmInstructionLabel.setText(algo + " : " + String.join(" -> ", traversalOrder));
                }
            } catch (InterruptedException | ExecutionException e) {
                e.printStackTrace();
                algorithmInstructionLabel.setText("An error occurred during traversal.");
            }
        }

        private class VertexDistancePair {
            VertexPanel vertex;
            int distance;

            public VertexDistancePair(VertexPanel vertex, int distance) {
                this.vertex = vertex;
                this.distance = distance;
            }
        }

        private class EdgeWeightPair {
            Edge edge;
            VertexPanel toVertex;
            int weight;

            public EdgeWeightPair(Edge edge, VertexPanel toVertex, int weight) {
                this.edge = edge;
                this.toVertex = toVertex;
                this.weight = weight;
            }
        }
    }
}