---
description: "L'opérateur findIndex est un opérateur de filtrage RxJS qui retourne l'index de la première valeur satisfaisant une condition. Il retourne -1 si non trouvé."
---

# findIndex - Obtenir Index Correspondant

L'opérateur `findIndex` retourne **l'index de la première valeur satisfaisant une condition** et termine immédiatement le flux. Si aucune valeur n'est trouvée, il retourne `-1`.

## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Sortie: 4 (index du premier nombre pair 8)
```

**Flux d'opération** :
1. 1 (index 0) → impair, ignoré
2. 3 (index 1) → impair, ignoré
3. 5 (index 2) → impair, ignoré
4. 7 (index 3) → impair, ignoré
5. 8 (index 4) → pair, émet l'index 4 et termine

[🌐 Documentation officielle RxJS - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Patterns d'utilisation typiques

- **Localisation de position dans un tableau** : Récupérer la position d'un élément satisfaisant une condition spécifique
- **Vérification d'ordre** : À quelle position apparaît un élément satisfaisant une condition
- **Tri de données** : Traitement utilisant les informations d'index
- **Vérification d'existence** : Confirmer l'existence en vérifiant si le résultat est -1

## 🧠 Exemple de code pratique 1 : Recherche dans une liste de tâches

Un exemple de recherche de la position de tâches correspondant à des conditions spécifiques dans une liste.

```ts
import { from, fromEvent } from 'rxjs';
import { findIndex } from 'rxjs';

interface Task {
  id: number;
  title: string;
  priority: 'high' | 'medium' | 'low';
  completed: boolean;
}

const tasks: Task[] = [
  { id: 1, title: 'Répondre aux emails', priority: 'low', completed: true },
  { id: 2, title: 'Créer des documents', priority: 'medium', completed: true },
  { id: 3, title: 'Préparer la réunion', priority: 'high', completed: false },
  { id: 4, title: 'Revue de code', priority: 'high', completed: false },
  { id: 5, title: 'Mettre à jour la documentation', priority: 'low', completed: false }
];

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Recherche de tâches';
container.appendChild(title);

// Affichage de la liste des tâches
const taskList = document.createElement('ul');
taskList.style.listStyle = 'none';
taskList.style.padding = '0';
tasks.forEach((task, index) => {
  const li = document.createElement('li');
  li.style.padding = '5px';
  li.style.borderBottom = '1px solid #eee';
  const status = task.completed ? '✅' : '⬜';
  const priorityBadge = task.priority === 'high' ? '🔴' : task.priority === 'medium' ? '🟡' : '🟢';
  li.textContent = `[${index}] ${status} ${priorityBadge} ${task.title}`;
  taskList.appendChild(li);
});
container.appendChild(taskList);

// Boutons de recherche
const buttonContainer = document.createElement('div');
buttonContainer.style.marginTop = '10px';
container.appendChild(buttonContainer);

const button1 = document.createElement('button');
button1.textContent = 'Rechercher la première tâche non terminée';
button1.style.marginRight = '10px';
buttonContainer.appendChild(button1);

const button2 = document.createElement('button');
button2.textContent = 'Rechercher la première tâche haute priorité';
buttonContainer.appendChild(button2);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Rechercher la première tâche non terminée
// Note : le pattern recommandé est d'aplatir avec `switchMap`,
// mais ici le subscribe est imbriqué pour la lisibilité (en production, préférez `switchMap`).
fromEvent(button1, 'click').subscribe(() => {
  // Subscription imbriquée : le pattern recommandé est l'aplatissement avec `switchMap`
  from(tasks).pipe(
    findIndex(task => !task.completed)
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Trouvé</strong><br>
        Position: Index ${index}<br>
        Tâche: ${task.title}<br>
        Priorité: ${task.priority}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Aucune tâche non terminée trouvée';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});

// Rechercher la première tâche haute priorité
// Note : le pattern recommandé est d'aplatir avec `switchMap` (en production, préférez `switchMap`).
fromEvent(button2, 'click').subscribe(() => {
  // Subscription imbriquée : le pattern recommandé est l'aplatissement avec `switchMap`
  from(tasks).pipe(
    findIndex(task => task.priority === 'high')
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Trouvé</strong><br>
        Position: Index ${index}<br>
        Tâche: ${task.title}<br>
        Statut: ${task.completed ? 'Terminé' : 'Non terminé'}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Aucune tâche haute priorité trouvée';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});
```

- Recherche la position de la première tâche correspondant à une condition dans la liste des tâches.
- Retourne `-1` si non trouvé.

## 🎯 Exemple de code pratique 2 : Détection de position dans des données temps réel

Un exemple de détection de la position de la première valeur correspondant à une condition dans un flux.

```ts
import { interval } from 'rxjs';
import { findIndex, map, take } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Recherche de données temps réel';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Recherche de la position de la première valeur >= 50...';
container.appendChild(status);

const dataDisplay = document.createElement('div');
dataDisplay.style.marginTop = '10px';
dataDisplay.style.padding = '10px';
dataDisplay.style.border = '1px solid #ccc';
dataDisplay.style.maxHeight = '150px';
dataDisplay.style.overflow = 'auto';
container.appendChild(dataDisplay);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.fontWeight = 'bold';
container.appendChild(result);

// Générer des valeurs aléatoires (0-100)
const data$ = interval(500).pipe(
  take(20),
  map(i => ({ index: i, value: Math.floor(Math.random() * 100) }))
);

// Affichage des données
data$.subscribe(data => {
  const div = document.createElement('div');
  const highlight = data.value >= 50 ? 'background-color: #fff9c4;' : '';
  div.style.cssText = `padding: 5px; ${highlight}`;
  div.textContent = `[${data.index}] Valeur: ${data.value}`;
  dataDisplay.appendChild(div);
  dataDisplay.scrollTop = dataDisplay.scrollHeight;
});

// Rechercher l'index de la première valeur >= 50
data$.pipe(
  findIndex(data => data.value >= 50)
).subscribe(index => {
  status.textContent = '';
  if (index !== -1) {
    result.innerHTML = `
      ✅ Valeur >= 50 trouvée<br>
      Position: Index ${index}
    `;
    result.style.color = 'green';
  } else {
    result.textContent = '❌ Aucune valeur >= 50 trouvée';
    result.style.color = 'orange';
  }
});
```

- Détecte la position de la première valeur >= 50 parmi des valeurs aléatoires générées toutes les 0.5 secondes.
- Mise en évidence visuelle pour une meilleure clarté.

## 🆚 Comparaison avec des opérateurs similaires

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: retourne l'index de la première valeur correspondante
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Sortie: 2 (index de 30)

// find: retourne la première valeur correspondante
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Sortie: 30

// elementAt: retourne la valeur à l'index spécifié
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Sortie: 30
```

| Opérateur | Argument | Valeur retournée | Si non trouvé |
|:---|:---|:---|:---|
| `findIndex(predicate)` | Fonction de condition | Index (nombre) | `-1` |
| `find(predicate)` | Fonction de condition | La valeur elle-même | `undefined` |
| `elementAt(index)` | Index | La valeur elle-même | Erreur (sans valeur par défaut) |

## 🔄 Comparaison avec Array.findIndex() de JavaScript

Le `findIndex` de RxJS fonctionne de manière similaire à la méthode `Array.prototype.findIndex()` de JavaScript.

```ts
// Tableau JavaScript
const numbers = [10, 20, 30, 40, 50];
const index1 = numbers.findIndex(n => n > 25);
console.log(index1); // 2

// Observable RxJS
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log); // 2
```

**Principales différences** :
- **Tableau** : Retourne le résultat immédiatement de manière synchrone
- **Observable** : Asynchrone, attend que les valeurs arrivent du flux

## ⚠️ Points d'attention

### 1. Retourne -1 si non trouvé

Si aucune valeur ne satisfait la condition, il retourne `-1` au lieu d'une erreur.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('Aucune valeur correspondante trouvée');
  } else {
    console.log(`Index: ${index}`);
  }
});
// Sortie: Aucune valeur correspondante trouvée
```

### 2. Termine immédiatement à la première découverte

Quand la première valeur correspondante est trouvée, le flux termine immédiatement.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Valeur: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Index: ${index}`);
});
// Sortie:
// Valeur: 0
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Index: 3
```

### 3. Sécurité de type avec TypeScript

`findIndex` retourne toujours un type `number`.

```ts
import { Observable, from } from 'rxjs';
import { findIndex } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

function findFirstInactiveUserIndex(
  users$: Observable<User>
): Observable<number> {
  return users$.pipe(
    findIndex(user => !user.isActive)
  );
}

const users$ = from([
  { id: 1, name: 'Alice', isActive: true },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true }
]);

findFirstInactiveUserIndex(users$).subscribe(index => {
  // index est de type number
  if (index !== -1) {
    console.log(`Le premier utilisateur inactif est à l'index ${index}`);
  }
});
// Sortie: Le premier utilisateur inactif est à l'index 1
```

### 4. Les index commencent à 0

Comme les tableaux, les index commencent à 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Sortie: 0 (premier élément)
```

## 📚 Opérateurs associés

- **[find](./find)** - Récupérer la première valeur correspondante
- **[elementAt](./elementAt)** - Récupérer la valeur à un index spécifié
- **[first](./first)** - Récupérer la première valeur
- **[filter](./filter)** - Récupérer toutes les valeurs correspondantes

## Résumé

L'opérateur `findIndex` retourne l'index de la première valeur satisfaisant une condition.

- ✅ Comportement similaire à `Array.findIndex()` de JavaScript
- ✅ Optimal quand l'information d'index est nécessaire
- ✅ Retourne `-1` si non trouvé (pas une erreur)
- ✅ Termine immédiatement à la découverte
- ⚠️ La valeur retournée est toujours de type `number` (-1 ou entier >= 0)
- ⚠️ Utilisez `find` si vous avez besoin de la valeur elle-même
