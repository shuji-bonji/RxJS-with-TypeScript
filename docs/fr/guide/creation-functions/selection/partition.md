---
description: "partition est une fonction de création RxJS qui divise un Observable en deux Observables en fonction des conditions. Elle est idéale pour les traitements de bifurcation tels que succès/échec, valide/invalide, etc."
---

# partition - diviser en deux flux selon une condition

`partition` est une fonction de création qui **divise** un Observable en deux Observables selon une condition.
Vous pouvez spécifier la condition avec une fonction prédicat (predicate) et obtenir les valeurs qui satisfont la condition et les valeurs qui ne satisfont pas la condition comme des flux séparés.

## Syntaxe de base et utilisation

```ts
import { partition, of } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Diviser en nombres pairs et impairs
const [evens$, odds$] = partition(source$, (value) => value % 2 === 0);

evens$.subscribe((value) => console.log('Pair:', value));
// Sortie: Pair: 2, Pair: 4, Pair: 6

odds$.subscribe((value) => console.log('Impair:', value));
// Sortie: Impair: 1, Impair: 3, Impair: 5
```

- `partition` renvoie un **tableau contenant deux Observables**.
- `[0]` : un flux de valeurs qui satisfont la condition.
- `[1]` : un flux de valeurs qui ne satisfont pas la condition.

[🌐 Documentation officielle RxJS - `partition`](https://rxjs.dev/api/index/function/partition)

## Modèles d'utilisation typiques

- **Traitement séparé des succès/échecs** (tri par code d'état HTTP)
- **Classification des événements** (clic gauche/clic droit)
- **Classification des données** (valide/invalide, adulte/enfant, etc.)
- **Division de flux selon des conditions**

## Exemples de code pratique (avec interface utilisateur)

Lorsqu'on clique sur un bouton, le processus est ramifié selon que les coordonnées du clic correspondent à la moitié gauche ou droite de l'écran.

```ts
import { partition, fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Créer une zone de sortie
const leftArea = document.createElement('div');
leftArea.innerHTML = '<h3>Clic gauche</h3><ul id="left-list"></ul>';
leftArea.style.float = 'left';
leftArea.style.width = '45%';
leftArea.style.padding = '10px';
leftArea.style.background = '#e3f2fd';
document.body.appendChild(leftArea);

const rightArea = document.createElement('div');
rightArea.innerHTML = '<h3>Clic droit</h3><ul id="right-list"></ul>';
rightArea.style.float = 'right';
rightArea.style.width = '45%';
rightArea.style.padding = '10px';
rightArea.style.background = '#fce4ec';
document.body.appendChild(rightArea);

// Événements de clic
const clicks$ = fromEvent<MouseEvent>(document, 'click');

// Coordonnée X centrale de l'écran
const centerX = window.innerWidth / 2;

// Diviser en moitié gauche et droite
const [leftClicks$, rightClicks$] = partition(
  clicks$,
  (event) => event.clientX < centerX
);

// Traiter les clics à gauche
leftClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const leftList = document.getElementById('left-list')!;
  const li = document.createElement('li');
  li.textContent = `Position: (${pos.x}, ${pos.y})`;
  leftList.appendChild(li);
});

// Traiter les clics à droite
rightClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const rightList = document.getElementById('right-list')!;
  const li = document.createElement('li');
  li.textContent = `Position: (${pos.x}, ${pos.y})`;
  rightList.appendChild(li);
});
```

- Les clics sur l'écran seront enregistrés dans les listes de gauche et de droite en fonction de la position du clic.
- Deux flux indépendants peuvent être créés à partir d'une seule source.

## Exemple pratique : Traitement des réponses API par branchement

Exemple de séparation du succès et de l'échec par code d'état HTTP

```ts
import { partition, from, of } from 'rxjs';
import { mergeMap, map, catchError, share } from 'rxjs';

interface ApiResponse {
  status: number;
  data?: any;
  error?: string;
}

// Appels API factices
const apiCalls$ = from([
  fetch('/api/users/1'),
  fetch('/api/users/999'), // Utilisateur inexistant
  fetch('/api/users/2'),
]);

// Traiter la réponse et convertir en ApiResponse
const responses$ = apiCalls$.pipe(
  mergeMap(fetchPromise => from(fetchPromise)),
  mergeMap(response =>
    from(response.json()).pipe(
      map(data => ({
        status: response.status,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : (data.message || 'Erreur')
      } as ApiResponse)),
      catchError(err => of({
        status: response.status,
        data: undefined,
        error: err.message || 'Échec de l\'analyse de la réponse'
      } as ApiResponse))
    )
  ),
  share() // Gérer 2 abonnements depuis partition
);

// Diviser en succès (200s) et échec (autres)
const [success$, failure$] = partition(
  responses$,
  (response: ApiResponse) => response.status >= 200 && response.status < 300
);

// Gérer les réponses de succès
success$.subscribe((response) => {
  console.log('✅ Succès:', response.data);
  // Afficher les données de succès dans l'interface utilisateur
});

// Gérer les réponses d'échec
failure$.subscribe((response) => {
  console.error('❌ Échec:', response.error);
  // Afficher le message d'erreur
});
```

## Comparaison avec filter

### Différences fondamentales

| Méthode | Description | Sortie | Cas d'utilisation |
|---------|-------------|--------|-------------------|
| `partition` | Diviser une source en deux flux | 2 Observables | Quand vous voulez utiliser les deux flux **simultanément** |
| `filter` | Ne passe que les valeurs qui remplissent la condition | 1 Observable | Quand un seul flux est nécessaire |

### Exemples d'utilisation

**Utiliser partition pour traiter les deux flux simultanément**

```ts
import { partition, interval } from 'rxjs';
import { map, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Succès</h4><ul id="success-list"></ul>';
successArea.style.float = 'left';
successArea.style.width = '45%';
output.appendChild(successArea);

const failureArea = document.createElement('div');
failureArea.innerHTML = '<h4 style="color: red;">❌ Échec</h4><ul id="failure-list"></ul>';
failureArea.style.float = 'right';
failureArea.style.width = '45%';
output.appendChild(failureArea);

// Flux de succès/échec aléatoire
const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Tâche ${i + 1}`
  }))
);

// ✅ partition - gérer succès et échec simultanément
const [success$, failure$] = partition(tasks$, task => task.success);

success$.subscribe(task => {
  const successList = document.getElementById('success-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  successList.appendChild(li);
});

failure$.subscribe(task => {
  const failureList = document.getElementById('failure-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  failureList.appendChild(li);
});
```

**Utiliser filter si un seul flux est nécessaire**

```ts
import { interval } from 'rxjs';
import { map, take, filter } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Afficher uniquement les succès</h4><ul id="success-only"></ul>';
output.appendChild(successArea);

const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Tâche ${i + 1}`
  }))
);

// ✅ filter - traiter uniquement les succès (ignorer les échecs)
tasks$
  .pipe(filter(task => task.success))
  .subscribe(task => {
    const successList = document.getElementById('success-only')!;
    const li = document.createElement('li');
    li.textContent = task.message;
    successList.appendChild(li);
  });
```

**Utiliser filter deux fois vs partition**

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// ❌ Utiliser filter deux fois - la source peut être exécutée deux fois
const evens1$ = numbers$.pipe(filter(n => n % 2 === 0));
const odds1$ = numbers$.pipe(filter(n => n % 2 !== 0));

evens1$.subscribe(n => console.log('Pair:', n));
odds1$.subscribe(n => console.log('Impair:', n));
// Problème: si numbers$ est un observable froid, il sera exécuté deux fois

// ✅ Utiliser partition - créer les deux flux en une seule exécution
const [evens2$, odds2$] = partition(numbers$, n => n % 2 === 0);

evens2$.subscribe(n => console.log('Pair:', n));
odds2$.subscribe(n => console.log('Impair:', n));
// Avantage: créer efficacement deux flux à partir d'une seule source
```

**Utiliser filter si vous voulez créer des branchements dans le pipeline**

```ts
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  age: number;
  isActive: boolean;
}

const users$ = from([
  { id: 1, name: 'Alice', age: 25, isActive: true },
  { id: 2, name: 'Bob', age: 30, isActive: false },
  { id: 3, name: 'Carol', age: 35, isActive: true }
]);

// ❌ partition est une fonction de création, elle ne peut pas être utilisée dans un pipeline
// users$.pipe(
//   map(user => user.name),
//   partition(name => name.startsWith('A')) // Erreur
// );

// ✅ Utiliser filter - disponible dans le pipeline
users$
  .pipe(
    filter(user => user.isActive),  // Uniquement les utilisateurs actifs
    map(user => user.name)           // Extraire le nom
  )
  .subscribe(console.log);
// Sortie: Alice, Carol
```

### Résumé

| Situation | Méthode recommandée | Raison |
|-----------|---------------------|--------|
| Vouloir traiter **les deux** succès et échec | `partition` | Peut créer deux flux en une seule exécution de la source |
| Vouloir traiter **uniquement** les succès | `filter` | Simple et clair |
| Vouloir créer des conditions de branchement dans le pipeline | `filter` | `partition` ne peut pas être utilisé car c'est une fonction de création |
| Vouloir créer 3 branches ou plus avec des conditions complexes | `groupBy` | Peut se diviser en plusieurs groupes |

## Notes

### 1. S'abonner aux deux flux

Les deux Observables créés dans une `partition` **partagent** la source originale.
Si vous ne vous abonnez pas aux deux, il se peut que le flux original ne soit pas entièrement traité.

```ts
const [success$, failure$] = partition(source$, predicate);

// S'abonner aux deux
success$.subscribe(handleSuccess);
failure$.subscribe(handleFailure);
```

### 2. La source est exécutée deux fois

La `partition` s'abonne en interne à la source originale deux fois.
Attention aux effets de bord.

```ts
let callCount = 0;
const source$ = new Observable(observer => {
  callCount++;
  console.log(`Nombre d'abonnements: ${callCount}`);
  observer.next(1);
  observer.complete();
});

const [a$, b$] = partition(source$, n => n > 0);
a$.subscribe(); // Nombre d'abonnements: 1
b$.subscribe(); // Nombre d'abonnements: 2
```

Pour éviter les effets de bord, utilisez `share()`.

```ts
import { share } from 'rxjs';

const shared$ = source$.pipe(share());
const [a$, b$] = partition(shared$, n => n > 0);
```

### 3. Non fourni en tant qu'opérateur Pipeable

Depuis RxJS 7, `partition` est fourni comme **fonction de création uniquement**.
Elle ne peut pas être utilisée dans un pipeline.

```ts
// ❌ Impossible
source$.pipe(
  partition(n => n % 2 === 0) // Erreur
);

// ✅ Utilisation correcte
const [evens$, odds$] = partition(source$, n => n % 2 === 0);
```

## Modèles alternatifs

Si vous voulez créer des branchements dans un pipeline, utilisez `filter`.

```ts
const source$ = of(1, 2, 3, 4, 5, 6);

const evens$ = source$.pipe(filter(n => n % 2 === 0));
const odds$ = source$.pipe(filter(n => n % 2 !== 0));

// Ou partager la source avec share
const shared$ = source$.pipe(share());
const evens$ = shared$.pipe(filter(n => n % 2 === 0));
const odds$ = shared$.pipe(filter(n => n % 2 !== 0));
```

## Opérateurs associés

- [`filter`](/fr/guide/operators/filtering/filter) - Ne passe que les valeurs qui satisfont une condition
- [`groupBy`](/fr/guide/operators/transformation/groupBy) - Divise en plusieurs groupes
- [`share`](/fr/guide/operators/multicasting/share) - Partage une source

## Résumé

`partition` est un outil puissant pour diviser un Observable en deux selon une condition.

- ✅ Idéal pour le traitement séparé succès/échec
- ✅ Crée deux flux indépendants
- ⚠️ Les sources sont souscrites deux fois (attention aux effets de bord)
- ⚠️ Non proposé en tant qu'opérateur Pipeable
