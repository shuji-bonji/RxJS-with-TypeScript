---
description: "L'opérateur findIndex est un opérateur de filtrage RxJS qui renvoie l'index de la première valeur qui satisfait la condition. S'il n'en trouve pas, il renvoie -1."
---

# findIndex - obtient l'index correspondant à la condition

L'opérateur `finIndex` retourne **l'index de la première valeur qui correspond à la condition** et complète le flux immédiatement. Il retourne `-1` si aucune valeur n'est trouvée.

## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Sortie.: 4(premier pair8indice du premier pair)
```

**Flux des opérations** :.
1. 1 (index 0) → impair, sauter
2. 3 (indice 1) → impair, sauter
3. 5 (index 2) → impair, sauter
4. 7 (indice 3) → impair, sauter
5. 8 (index 4) → nombre pair, sortie de l'index 4 et terminé

[🌐 Official RxJS documentation - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Modèle d'utilisation typique.

- **Positionnement dans un tableau** : obtenir la position d'un élément qui satisfait une condition spécifique.
- **Vérification de l'ordre** : combien de fois apparaît un élément satisfaisant une certaine condition.
- Ranger les données** : traitement utilisant les informations d'index.
- Contrôle d'existence** : contrôle de l'existence d'un élément en vérifiant s'il est -1 ou non.

## 🧠 Exemple de code pratique 1 : Recherche dans une liste de tâches

Voici un exemple de recherche de l'emplacement d'une tâche avec des conditions spécifiques à partir d'une liste de tâches.

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
  { id: 1, title: 'Réponse au courrier électronique', priority: 'low', completed: true },
  { id: 2, title: 'Préparation d'un document', priority: 'medium', completed: true },
  { id: 3, title: 'Préparation de réunion', priority: 'high', completed: false },
  { id: 4, title: 'Révision du code', priority: 'high', completed: false },
  { id: 5, title: 'Mise à jour du document', priority: 'low', completed: false }
];

// UICréer
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

// Bouton de recherche
const buttonContainer = document.createElement('div');
buttonContainer.style.marginTop = '10px';
container.appendChild(buttonContainer);

const button1 = document.createElement('button');
button1.textContent = 'Recherche de la première tâche inachevée';
button1.style.marginRight = '10px';
buttonContainer.appendChild(button1);

const button2 = document.createElement('button');
button2.textContent = 'Recherche de la première tâche prioritaire';
buttonContainer.appendChild(button2);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Recherche de la première tâche inachevée
// NB.: À l'origine, le schéma recommandé était d'aplatir avec switchMap Le modèle recommandé est d'aplatir avec
// Ici, la priorité en matière de lisibilité est donnée aux tâches subscribe imbriqué (dans le code de production switchMap recommandé).
fromEvent(button1, 'click').subscribe(() => {
  // L'imbrication subscribe: À l'origine, le schéma recommandé était d'aplatir avec switchMap Il est recommandé d'aplatir avec
  from(tasks).pipe(
    findIndex(task => !task.completed)
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Trouvé dans</strong><br>
        Position: Index ${index}<br>
        Tâche: ${task.title}<br>
        Priorité: ${task.priority}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Tâche inachevée non trouvée';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});

// Recherche de la première tâche prioritaire
// NB.: À l'origine, le schéma recommandé était d'aplatir avec switchMap Le modèle recommandé (dans le code de production) est d'aplatir avec switchMap recommandé).
fromEvent(button2, 'click').subscribe(() => {
  // L'imbrication subscribe: À l'origine, le schéma recommandé était d'aplatir avec switchMap Il est recommandé d'aplatir avec
  from(tasks).pipe(
    findIndex(task => task.priority === 'high')
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Trouvé dans</strong><br>
        Position: Index ${index}<br>
        Tâche: ${task.title}<br>
        Statut d'achèvement: ${task.completed ? 'Terminée' : 'Inachevée'}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Aucune tâche prioritaire n'a été trouvée';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});
```

- Trouve la position de la première tâche de la liste des tâches qui remplit la condition.
- Si elle n'est pas trouvée, elle renvoie `-1`.

## 🎯 Exemple de code pratique 2 : Détection de l'emplacement des données en temps réel

Cet exemple détecte la position de la première valeur du flux qui satisfait à la condition.

```ts
import { interval } from 'rxjs';
import { findIndex, map, take } from 'rxjs';

// UICréer
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Recherche de données en temps réel';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '50Recherche de positions dont la valeur est supérieure ou égale à...';
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

// Génération de valeurs aléatoires (0~100)
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

// 50Recherche dans l'index de la première valeur supérieure à
data$.pipe(
  findIndex(data => data.value >= 50)
).subscribe(index => {
  status.textContent = '';
  if (index !== -1) {
    result.innerHTML = `
      ✅ 50Valeur supérieure ou égale trouvée<br>
      Position: Index ${index}
    `;
    result.style.color = 'green';
  } else {
    result.textContent = '❌ 50Aucune valeur supérieure ou égale n'a été trouvée';
    result.style.color = 'orange';
  }
});
```

- Détecte la position de la première valeur supérieure à 50 à partir de valeurs aléatoires générées toutes les 0,5 secondes.
- Le surlignage est utilisé pour la clarté visuelle.

## 🆚 Comparaison avec des opérateurs similaires

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Renvoie l'index de la première valeur qui satisfait à la condition
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Sortie.: 2Renvoie l'index de la première valeur qui satisfait à la condition30indice du premier pair)

// find: Renvoie la première valeur qui remplit la condition
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Sortie.: 30

// elementAt: Renvoie la valeur à l'index spécifié
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Sortie.: 30
```

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Sortie.: 4(premier pair8indice du premier pair)
```

## 🔄 Comparaison avec la fonction Array.findIndex() de JavaScript

RxJS `findIndex` se comporte de manière similaire à la méthode de tableau JavaScript `Array.prototype.findIndex()`.


```ts
// JavaScript Tableau de
const numbers = [10, 20, 30, 40, 50];
const index1 = numbers.findIndex(n => n > 25);
console.log(index1); // 2

// RxJS (renvoie la première valeur à l'index spécifié qui remplit la condition) Observable
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log); // 2
```

**Principales différences**.
- **Tableau** : renvoie le résultat de manière synchrone et immédiate.
- **Observable** : asynchrone, attend que les valeurs s'écoulent du flux.

## ⚠️ Notes.

### 1. renvoie -1 si non trouvé

Si aucune valeur ne satisfait la condition, renvoie `-1` au lieu d'une erreur.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('Aucune valeur satisfaisant la condition n'a été trouvée');
  } else {
    console.log(`Index: ${index}`);
  }
});
// Sortie.: Aucune valeur satisfaisant la condition n'a été trouvée
```

### 2. complet dès la première découverte.

Le flux est complété dès que la première valeur satisfaisant la condition est trouvée.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Valeur: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Index: ${index}`);
});
// Sortie.:
// Valeur: 0
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Index: 3
```

### 3. la sécurité de type dans TypeScript

`finIndex` renvoie toujours le type `number`.

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
  // index est un tableau de number type
  if (index !== -1) {
    console.log(`Le premier utilisateur inactif est à l'index ${index} est.`);
  }
});
// Sortie.: Le premier utilisateur inactif est à l'index 1 est.
```

### 4. L'index commence à 0

Comme pour les tableaux, les index commencent à 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Sortie.: 0(premier élément)
```

## 📚 Opérateurs apparentés.

- **[find](. /find)** - Obtenir la première valeur qui satisfait la condition.
- **[elementAt](. /elementAt)** - Obtient la valeur à l'index spécifié.
- **[first](. /first)** - Obtient la première valeur.
- **[filter](. /filter)** - Récupère toutes les valeurs qui satisfont à la condition.

## Résumé.

L'opérateur findIndex renvoie l'index de la première valeur qui satisfait la condition.

- ✅ Comportement similaire à celui du JavaScript `Array.findIndex()`.
- ✅ Idéal lorsque l'information sur l'index est nécessaire
- ✅ Retourne `-1` si non trouvé (pas une erreur)
- ✅ Complète immédiatement lorsqu'elle est trouvée
- ⚠️ La valeur de retour est toujours de type `number` (-1 ou un entier supérieur ou égal à 0)
- ⚠️ Utiliser `find` si la valeur elle-même est requise
