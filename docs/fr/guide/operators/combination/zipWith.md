---
description: "zipWith est un opérateur de jointure RxJS qui aligne et associe l'Observable d'origine avec les valeurs ordonnées correspondantes provenant d'autres Observables. Il est idéal pour les jointures de flux critiques en termes d'ordre, telles que la combinaison des résultats du traitement parallèle avec des garanties d'ordre, la mise en correspondance des ID avec les données, la synchronisation des données connexes publiées à des moments différents, etc. La version de l'opérateur pipeable est pratique pour une utilisation au sein des pipelines."
---

# zipWith - paire de valeurs correspondantes

L'opérateur `zipWith` regroupe les **valeurs ordonnées correspondantes** émises par l'Observable original et les autres Observables spécifiés.
Il attend que les valeurs arrivent une par une de tous les Observable et crée des paires lorsqu'elles sont prêtes.
C'est la version Pipeable Operator du `zip` de Creation Function.

## 🔰 Syntaxe de base et utilisation

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

// La sortie:
// A - 0
// B - 10
// C - 20
// (Dn'est pas une sortie car il n'y a pas de valeur correspondante)
```

- Une paire est produite par chaque Observable **quand l'une des valeurs est complète**.
- Lorsqu'un Observable est complet, les valeurs restantes sont rejetées.

[🌐 Official RxJS documentation - `zipWith`](https://rxjs.dev/api/operators/zipWith)

## 💡 Modèle d'utilisation typique.

- **Combinaison des résultats de traitement parallèle dans un ordre garanti** : couplage des résultats de plusieurs appels API.
- **Mapping IDs to data** : combinaison des IDs des utilisateurs avec les données de profil correspondantes.
- Synchronisation des flux** : synchronisation de données connexes émises à des moments différents.

## 🧠 Exemples de code pratique (avec interface utilisateur)

Exemple d'appariement d'une liste d'identifiants d'utilisateurs et de noms d'utilisateurs correspondants dans l'ordre.

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// Création d'une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>zipWith Exemples pratiques de:</h3>';
document.body.appendChild(output);

// L'utilisateurIDStream (publié immédiatement)
const userIds$ = from([101, 102, 103, 104]);

// Flux de noms d'utilisateurs (publié toutes les1(publié toutes les secondes)
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// zipet affiché
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 L'utilisateurID ${id}: ${name}`;
    output.appendChild(item);
  });

// La sortie:
// 👤 L'utilisateurID 101: Alice
// 👤 L'utilisateurID 102: Bob
// 👤 L'utilisateurID 103: Carol
// (104non affiché car il n'y a pas de nom correspondant)
```

- Les identifiants et les noms sont appariés dans une correspondance **un pour un**.
- Lorsqu'une valeur est complétée, les valeurs restantes sont rejetées.

## 🔄 Différence par rapport à la Creation Function `zip`.

### Différences fondamentales.

|  | `zip` (Creation Function). | `zipWith` (Pipeable Operator). |
|---|---|---|
| **Où utilisé** | Utilisée comme fonction autonome | Utilisée dans la chaîne `.pipe()`. |
| **Comment écrire**. | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Premier flux**. | Traiter tous les flux comme égaux | Traiter comme le flux principal |
| **Avantages**. | Simple et facile à lire | Facile à combiner avec d'autres opérateurs |

### Exemples spécifiques d'utilisation

**Pour un appariement simple, la Creation Function est recommandée**.

```ts
import { zip, of } from 'rxjs';

const questions$ = of('Le nom est？', 'L'âge est？', 'L'adresse est？');
const answers$ = of('Taro', '30', 'Tokyo');
const scores$ = of(10, 20, 30);

// Simple et lisible
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`Q: ${q}, A: ${a}, Score: ${s}Score`);
});
// La sortie:
// Q: Le nom est？, A: Taro, Score: 10Score
// Q: L'âge est？, A: 30, Score: 20Score
// Q: L'adresse est？, A: Tokyo, Score: 30Score
```

**Si vous souhaitez ajouter un processus de transformation au flux principal, le Pipeable Operator est recommandé**.

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// Liste des tâches
const tasks$ = from([
  { id: 1, name: 'Rapports', priority: 'high' },
  { id: 2, name: 'Réponse aux courriels', priority: 'low' },
  { id: 3, name: 'Préparation de réunions', priority: 'high' },
  { id: 4, name: 'Organiser les documents', priority: 'medium' }
]);

// Liste des responsables (1Attribuée toutes les secondes)
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Pipeable OperatorÉditions - Compléter en une seule fois
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // Priorité élevée uniquement
    map(task => task.name),                     // Extraire les noms des tâches
    zipWith(assignees$),                        // Assigner une personne en charge
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → Affecté à: ${assignment.assignee}`);
  });
// La sortie:
// [Temps] Rapports → Affecté à: Alice
// [Temps] Préparation de réunions → Affecté à: Bob

// ❌ Creation FunctionÉditions - Redondant
import { zip } from 'rxjs';
zip(
  tasks$.pipe(
    filter(task => task.priority === 'high'),
    map(task => task.name)
  ),
  assignees$
).pipe(
  map(([taskName, assignee]) => ({
    task: taskName,
    assignee,
    assignedAt: new Date().toLocaleTimeString()
  }))
).subscribe(assignment => {
  console.log(`[${assignment.assignedAt}] ${assignment.task} → Affecté à: ${assignment.assignee}`);
});
```

**Synchronisation des données critiques pour la commande**.

```ts
import { from } from 'rxjs';
import { zipWith, map, concatMap, delay } from 'rxjs';
import { of } from 'rxjs';

// UICréer
const output = document.createElement('div');
output.innerHTML = '<h3>Jeu-concours</h3>';
document.body.appendChild(output);

const questionArea = document.createElement('div');
questionArea.style.marginTop = '10px';
output.appendChild(questionArea);

// Liste de questions (préparée immédiatement)
const questions$ = from([
  'La capitale du Japon est？',
  '1+1est？',
  'Quelle est la planète Terre ?？'
]);

// Liste de réponses (simulant la saisie de l'utilisateur)：2(toutes les secondes)
const answers$ = from(['Tokyo', '2', '3']).pipe(
  concatMap((answer, index) =>
    of(answer).pipe(delay((index + 1) * 2000))
  )
);

// Liste des réponses correctes
const correctAnswers$ = from(['Tokyo', '2', '3']);

// ✅ Pipeable OperatorÉditions - Traiter les questions comme des questions courantes
questions$
  .pipe(
    zipWith(answers$, correctAnswers$),
    map(([question, answer, correct], index) => ({
      no: index + 1,
      question,
      answer,
      correct,
      isCorrect: answer === correct
    }))
  )
  .subscribe(result => {
    const div = document.createElement('div');
    div.style.marginTop = '10px';
    div.style.padding = '10px';
    div.style.border = '1px solid #ccc';
    div.style.backgroundColor = result.isCorrect ? '#e8f5e9' : '#ffebee';
    div.innerHTML = `
      <strong>Question${result.no}:</strong> ${result.question}<br>
      <strong>Réponses:</strong> ${result.answer}<br>
      <strong>Résultat:</strong> ${result.isCorrect ? '✅ Réponse correcte！' : '❌ Réponse incorrecte'}
    `;
    questionArea.appendChild(div);
  });
```

### Résumé.

- **`zip`** : idéal si vous souhaitez simplement mettre en correspondance plusieurs flux dans l'ordre.
- **`zipWith`** : idéal si vous voulez fusionner le flux principal avec d'autres flux dans un ordre garanti tout en transformant ou en traitant le flux principal.

## ⚠️ Notes.

### Pour des longueurs différentes.

Lorsque l'Observable le plus court se termine, les valeurs restantes de l'Observable le plus long sont rejetées.

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// La sortie: [1, 'A'], [2, 'B'], [3, 'C']
// 'D'et'E'sont rejetés
```

### Accumulation de mémoire.

Si un Observable continue à émettre des valeurs, celles-ci s'accumuleront dans la mémoire jusqu'à ce que l'autre le rattrape.

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// Flux rapides (100mspar flux)
const fast$ = interval(100).pipe(take(10));

// Flux à faible vitesse (1(toutes les secondes)
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// La sortie: [0, 0] (1secondes plus tard), [1, 1] (2secondes plus tard), [2, 2] (3secondes plus tard)
// fast$les valeurs sont stockées en mémoire et attendues
```

### Différence par rapport à combineLatestWith.

`zipWith` associe les valeurs dans l'ordre correspondant, alors que `combineLatestWith` combine les dernières valeurs.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: Appariées dans l'ordre correspondant
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// La sortie: [0, 0], [1, 1]

// combineLatestWith: Combinaison des dernières valeurs
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// La sortie: [0, 0], [1, 0], [2, 0], [2, 1]
```

## 📚 Opérateurs apparentés.

- **[zip](/fr/guide/creation-functions/combination/zip)** - Version de la Creation Function.
- **[combineLatestWith](/fr/guide/operators/combination/combineLatestWith)** - Combiner la valeur la plus récente.
- **[withLatestFrom](/fr/guide/operators/combination/withLatestFrom)** - Déclencheurs principaux uniquement
