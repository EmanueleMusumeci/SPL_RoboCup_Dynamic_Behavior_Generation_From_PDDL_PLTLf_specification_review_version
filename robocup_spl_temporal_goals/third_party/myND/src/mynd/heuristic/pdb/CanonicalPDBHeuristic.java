package mynd.heuristic.pdb;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import mynd.Global;
import mynd.MyNDPlanner;
import mynd.explicit.ExplicitCondition;
import mynd.heuristic.Heuristic;
import mynd.state.Condition;
import mynd.state.State;
import mynd.symbolic.BeliefState;
import mynd.symbolic.SymbolicCondition;
import mynd.util.HeuristicValueAggregation;


/**
 *
 * @author Manuela Ortlieb
 * @author Robert Mattmüller
 *
 */
public class CanonicalPDBHeuristic extends Heuristic {

  /**
   * Mapping from the pattern to its pattern database. Only patterns which are used in canonical
   * heuristic function.
   */
  public Map<Set<Integer>, PDB> patterntoPDB;

  /**
   * PDBs only stored temporarily when looking for best successor. After the pattern search these
   * pattern databases could be deleted.
   */
  public Map<Set<Integer>, PDB> temporaryPDBs;

  /**
   * Only one graph is needed for a collection of patterns.
   */
  public CompatibilityGraph compatibilityGraph;

  /**
   * All maximal cliques of a pattern collection.
   */
  public Set<Set<Set<Integer>>> maximalCliques;

  /**
   * Number of abstract states of given pattern collection.
   */
  int size;

  /**
   * Sum of all abstract states in all temporary pdbs.
   */
  public int sizesOfTemporaryPDBs = 0;

  /**
   * Goal condition.
   */
  Condition goal;

  /**
   * Set true for debug output information.
   */
  public static final boolean DEBUG = false;

  /**
   * Constructor
   *
   * @param problem The underlying planning problem
   * @param patternCollection The pattern collection on which this heuristic is based.
   */
  public CanonicalPDBHeuristic(Set<Set<Integer>> patternCollection) {
    super(true); // Canonical PDB-heuristic supports axioms.
    if (PDB.buildExplicitPDBs) {
      goal = Global.problem.explicitGoal;
    } else {
      goal = Global.problem.getGoal();
    }
    initialize(patternCollection);
  }

  public CanonicalPDBHeuristic(Set<Set<Integer>> patternCollection, Condition goal) {
    super(true); // Canonical PDB-heuristic supports axioms.
    this.goal = goal;
    initialize(patternCollection);
  }

  private void initialize(Set<Set<Integer>> patternCollection) {
    if (DEBUG) {
      System.out.println("Constructor of CanonicalPDBHeuristic called.");
    }
    patterntoPDB = new HashMap<Set<Integer>, PDB>(20);
    temporaryPDBs = new HashMap<Set<Integer>, PDB>(200);
    fillPDBs(patternCollection);
    compatibilityGraph = new CompatibilityGraph(patternCollection);
    maximalCliques = compatibilityGraph.getMaximalCliques();
    // calculate size
    size = 1;
    for (Set<Integer> pattern : patternCollection) {
      size += PDB.numAbstractStates(pattern);
    }
  }

  public void addPatternToPatternCollection(Set<Integer> newPattern) {
    System.out.println("New Pattern is " + newPattern + ".");
    assert (!patterntoPDB.containsKey(newPattern));
    PDB newPDB;
    if (!PatternCollectionSearch.cacheTemporaryPDBs) {
      if (Global.problem.isFullObservable || MyNDPlanner.assumeFullObservability
          || PatternCollectionSearch.fullObservablePatternSearch) {
        newPDB = new ExplicitStatePDB(newPattern, (ExplicitCondition) goal);
      } else {
        newPDB = new BeliefStatePDB(newPattern, (SymbolicCondition) goal);
      }
    } else {
      newPDB = temporaryPDBs.get(newPattern);
    }
    patterntoPDB.put(newPattern, newPDB);
    compatibilityGraph.extendCompatibilityGraph(newPattern);
    maximalCliques = compatibilityGraph.getMaximalCliques();
    size += PDB.numAbstractStates(newPattern);
  }

  /**
   * Temporarily add a new PDB for a given pattern.
   */
  public boolean addTemporaryPatternDatabase(Set<Integer> newPattern) {
    assert !patterntoPDB.containsKey(newPattern);
    if (temporaryPDBs.containsKey(newPattern)) {
      return false;
    }
    if (Global.problem.isFullObservable || MyNDPlanner.assumeFullObservability
        || PatternCollectionSearch.fullObservablePatternSearch) {
      temporaryPDBs.put(newPattern, new ExplicitStatePDB(newPattern, (ExplicitCondition) goal));
    } else {
      temporaryPDBs.put(newPattern, new BeliefStatePDB(newPattern, (SymbolicCondition) goal));
    }
    sizesOfTemporaryPDBs += PDB.numAbstractStates(newPattern);
    return true;
  }

  /**
   * Deletes maximal cliques with dominated sums.
   *
   * Makes sure that <tt>maxcliques</tt> contains the up-to-date maximal cliques.
   */
  public void dominancePruning() {
    System.out.println("Number of maximal cliques before dominance pruning: "
        + maximalCliques.size());

    Set<Set<Set<Integer>>> allMaximalCliques = new HashSet<Set<Set<Integer>>>(maximalCliques);

    nextCandidateDominatedClique: for (Set<Set<Integer>> candidateDominatedClique : allMaximalCliques) {
      for (Set<Set<Integer>> candidateDominatingClique : allMaximalCliques) {
        if (candidateDominatingClique.equals(candidateDominatedClique)) {
          continue;
        }

        boolean allPatternsDominated = true;
        nextCandidateDominatedPattern: for (Set<Integer> candidateDominatedPattern : candidateDominatedClique) {
          for (Set<Integer> candidateDominatingPattern : candidateDominatingClique) {
            if (candidateDominatingPattern.containsAll(candidateDominatedPattern)) {
              continue nextCandidateDominatedPattern;
            }
          }
          allPatternsDominated = false;
          break;
        }
        if (allPatternsDominated) {
          maximalCliques.remove(candidateDominatedClique);
          continue nextCandidateDominatedClique;
        }
      }
    }
    System.out.println("Number of maximal cliques after dominance pruning: "
        + maximalCliques.size());
  }

  private void fillPDB(Set<Integer> pattern) {
    assert (!patterntoPDB.containsKey(pattern));
    if (DEBUG) {
      System.out.println("Pattern added: " + pattern);
    }
    if (PDB.buildExplicitPDBs) {
      patterntoPDB.put(pattern, new ExplicitStatePDB(pattern, (ExplicitCondition) goal));
    } else {
      patterntoPDB.put(pattern, new BeliefStatePDB(pattern, (SymbolicCondition) goal));
    }
  }

  private void fillPDBs(Set<Set<Integer>> patternCollection) {
    if (DEBUG) {
      System.out.println("Fill PDBs.");
    }
    for (Set<Integer> pattern : patternCollection) {
      fillPDB(pattern);
    }
    if (DEBUG) {
      System.out.println("PDBs filled.");
    }
  }

  double getCanonicalHeuristic(State s) {
    double maxH = Double.NEGATIVE_INFINITY;
    for (Set<Set<Integer>> clique : maximalCliques) {
      double current = 0;
      for (Set<Integer> pattern : clique) {
        double patternHeuristicValue = patterntoPDB.get(pattern).getHeuristic(s);
        if (patternHeuristicValue == Heuristic.INFINITE_HEURISTIC) {
          return patternHeuristicValue;
        }
        current += patternHeuristicValue;
      }
      if (current > maxH) {
        maxH = current;
      }
    }
    return maxH;
  }

  /**
   * Get heuristic value for given state.
   *
   * @param state state to be evaluated by heuristic
   * @return heuristic value for given state
   */
  @Override
  public double getHeuristic(State state) {
    if (MyNDPlanner.assumeFullObservability && state instanceof BeliefState) {
      // In this case we have to draw samples because all PDBs require explicit state.

      // Aggregate heuristic value of given belief state.
      switch (Heuristic.heuristicStrategy) {
        case ADD:
          return HeuristicValueAggregation.add(
              ((BeliefState) state).getRandomExplicitWorldStates(numberOfWorldStatesToBeSampled),
              this);
        case AVERAGE:
          return HeuristicValueAggregation.average(
              ((BeliefState) state).getRandomExplicitWorldStates(numberOfWorldStatesToBeSampled),
              this);
        case MAX:
          return HeuristicValueAggregation.maximize(
              ((BeliefState) state).getRandomExplicitWorldStates(numberOfWorldStatesToBeSampled),
              this);
        default:
          assert false;
          return -1;
      }
    }
    return getCanonicalHeuristic(state);
  }

  @Override
  public String toString() {
    StringBuffer buffer = new StringBuffer();
    buffer.append("Patterns: ");
    for (Set<Integer> pattern : patterntoPDB.keySet()) {
      buffer.append("{ ");
      for (int var : pattern) {
        buffer.append(var);
        buffer.append(" ");
      }
      buffer.append("} ");
    }
    buffer.append("\n");
    buffer.append("Maximal cliques: " + maximalCliques);
    return buffer.toString();
  }
}
