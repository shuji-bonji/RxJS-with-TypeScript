---
description: "zipWith est un opérateur de combinaison RxJS qui apparie les valeurs correspondantes de l'Observable original et d'autres Observables. Idéal pour combiner des résultats de traitements parallèles avec garantie d'ordre, associer des IDs à des données, synchroniser des données liées émises à des moments différents. Version pipeable operator pratique à utiliser dans les pipelines."
---

# zipWith - Paires Correspondantes

L'opérateur `zipWith` émet ensemble les **valeurs correspondantes** de l'Observable original et des autres Observables spécifiés.
Il attend qu'une valeur arrive de chaque Observable, puis crée une paire quand elles sont alignées.
C'est la version Pipeable Operator de la Creation Function `zip`.

## Syntaxe de base et utilisation

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const letters$ = of('A', 'B', 'C', 'D');
const numbers$ = interval(1000).pipe(
  map(val => val * 10),
  take(3)
);

letters$
  .pipe(zipWith(numbers$))
  .subscribe(([letter, number]) => {
    console.log(`${letter} - ${number}`);
  });

// Sortie:
// A - 0
// B - 10
// C - 20
// (D n'est pas émis car pas de valeur correspondante)
```

- Les paires sont émises **quand une valeur de chaque Observable est alignée**.
- Quand un Observable se termine, les valeurs restantes sont abandonnées.

[Documentation officielle RxJS - `zipWith`](https://rxjs.dev/api/operators/zipWith)


## Patterns d'utilisation typiques

- **Combiner des résultats de traitements parallèles avec garantie d'ordre**: Appairer des résultats de plusieurs appels API
- **Associer des IDs à des données**: Combiner des IDs utilisateur avec les données de profil correspondantes
- **Synchronisation de flux**: Synchroniser des données liées émises à des moments différents


## Exemple de code pratique (avec UI)

Un exemple qui apparie et affiche une liste d'IDs utilisateur avec les noms correspondants.

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de zipWith:</h3>';
document.body.appendChild(output);

// Flux d'IDs utilisateur (émis immédiatement)
const userIds$ = from([101, 102, 103, 104]);

// Flux de noms utilisateur (émis toutes les 1 seconde)
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// Appairer avec zip et afficher
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 ID utilisateur ${id}: ${name}`;
    output.appendChild(item);
  });

// Sortie:
// 👤 ID utilisateur 101: Alice
// 👤 ID utilisateur 102: Bob
// 👤 ID utilisateur 103: Carol
// (104 n'est pas affiché car pas de nom correspondant)
```

- Les IDs et noms sont **appariés 1 pour 1**.
- Quand l'un se termine, les valeurs restantes sont abandonnées.


## Différence avec la Creation Function `zip`

### Différences de base

| | `zip` (Creation Function) | `zipWith` (Pipeable Operator) |
|:---|:---|:---|
| **Lieu d'utilisation** | Utilisé comme fonction indépendante | Utilisé dans la chaîne `.pipe()` |
| **Syntaxe** | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Premier flux** | Traité également | Traité comme flux principal |
| **Avantage** | Simple et lisible | Facile à combiner avec d'autres opérateurs |

### Exemples concrets de choix

**Pour un simple appariement, la Creation Function est recommandée**

```ts
import { zip, of } from 'rxjs';

const questions$ = of('Votre nom?', 'Votre âge?', 'Votre adresse?');
const answers$ = of('Taro', '30', 'Tokyo');
const scores$ = of(10, 20, 30);

// Simple et lisible
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`Q: ${q}, R: ${a}, Score: ${s} points`);
});
// Sortie:
// Q: Votre nom?, R: Taro, Score: 10 points
// Q: Votre âge?, R: 30, Score: 20 points
// Q: Votre adresse?, R: Tokyo, Score: 30 points
```

**Pour ajouter des transformations au flux principal, le Pipeable Operator est recommandé**

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// Liste de tâches
const tasks$ = from([
  { id: 1, name: 'Créer rapport', priority: 'high' },
  { id: 2, name: 'Répondre emails', priority: 'low' },
  { id: 3, name: 'Préparer réunion', priority: 'high' },
  { id: 4, name: 'Organiser documents', priority: 'medium' }
]);

// Liste des assignés (assignés toutes les 1 seconde)
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Version Pipeable Operator - complète en un seul pipeline
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // Haute priorité uniquement
    map(task => task.name),                     // Extraire le nom de tâche
    zipWith(assignees$),                        // Assigner les responsables
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → Assigné: ${assignment.assignee}`);
  });
// Sortie:
// [heure] Créer rapport → Assigné: Alice
// [heure] Préparer réunion → Assigné: Bob
```

### Résumé

- **`zip`**: Optimal pour simplement appairer plusieurs flux par ordre
- **`zipWith`**: Optimal quand vous voulez combiner avec d'autres flux avec garantie d'ordre tout en ajoutant des transformations au flux principal


## Points d'attention

### Cas de longueurs différentes

Quand l'Observable le plus court se termine, les valeurs restantes de l'autre sont abandonnées.

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// Sortie: [1, 'A'], [2, 'B'], [3, 'C']
// 'D' et 'E' sont abandonnés
```

### Accumulation en mémoire

Si un Observable continue d'émettre des valeurs, elles s'accumulent en mémoire jusqu'à ce que l'autre rattrape.

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// Flux rapide (toutes les 100ms)
const fast$ = interval(100).pipe(take(10));

// Flux lent (toutes les 1 seconde)
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// Sortie: [0, 0] (après 1s), [1, 1] (après 2s), [2, 2] (après 3s)
// Les valeurs de fast$ s'accumulent en mémoire en attendant
```

### Différence avec combineLatestWith

`zipWith` apparie par ordre correspondant, tandis que `combineLatestWith` combine les dernières valeurs.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: Apparie par ordre correspondant
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Sortie: [0, 0], [1, 1]

// combineLatestWith: Combine les dernières valeurs
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Sortie: [0, 0], [1, 0], [2, 0], [2, 1]
```


## Opérateurs associés

- **[zip](/fr/guide/creation-functions/combination/zip)** - Version Creation Function
- **[combineLatestWith](/fr/guide/operators/combination/combineLatestWith)** - Combine les dernières valeurs
- **[withLatestFrom](/fr/guide/operators/combination/withLatestFrom)** - Seul le flux principal déclenche

