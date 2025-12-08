---
description: "find est un opérateur de filtrage RxJS qui trouve et émet la première valeur satisfaisant une condition, puis termine immédiatement le flux. Idéal pour la recherche d'utilisateurs, la vérification de stock, la détection d'erreurs dans les logs. Retourne undefined si non trouvé, avec un type de retour T | undefined en TypeScript."
---

# find - Trouver la première valeur satisfaisant une condition

L'opérateur `find` **trouve la première valeur** satisfaisant une condition, l'émet, puis termine immédiatement le flux. Si aucune valeur n'est trouvée, il émet `undefined`.


## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Sortie: 8 (premier nombre pair)
```

**Flux d'opération** :
1. 1, 3, 5, 7 vérifiés → ne satisfont pas la condition
2. 8 vérifié → satisfait la condition → émet 8 et termine
3. 9, 10 ne sont pas évalués

[🌐 Documentation officielle RxJS - `find`](https://rxjs.dev/api/operators/find)


## 🆚 Comparaison avec first

`find` et `first` sont similaires mais ont des usages différents.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: première valeur (condition optionnelle)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Sortie: 7

// find: première valeur (condition requise)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Sortie: 7
```

| Opérateur | Condition | Si non trouvé | Cas d'utilisation |
|---|---|---|---|
| `first()` | Optionnelle | Erreur (`EmptyError`) | Récupérer la première valeur |
| `first(predicate)` | Optionnelle | Erreur (`EmptyError`) | Récupération conditionnelle |
| `find(predicate)` | Requise | Émet `undefined` | Recherche/vérification d'existence |


## 💡 Patterns d'utilisation typiques

1. **Recherche d'utilisateur**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface User {
     id: number;
     name: string;
     email: string;
   }

   const users$ = from([
     { id: 1, name: 'Alice', email: 'alice@example.com' },
     { id: 2, name: 'Bob', email: 'bob@example.com' },
     { id: 3, name: 'Charlie', email: 'charlie@example.com' }
   ] as User[]);

   // Rechercher l'utilisateur avec l'ID 2
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Trouvé: ${user.name}`);
     } else {
       console.log('Utilisateur non trouvé');
     }
   });
   // Sortie: Trouvé: Bob
   ```

2. **Vérification de stock**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'PC portable', stock: 0 },
     { id: 'A2', name: 'Souris', stock: 15 },
     { id: 'A3', name: 'Clavier', stock: 8 }
   ] as Product[]);

   // Trouver le produit en rupture de stock
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Rupture de stock: ${product.name}`);
     } else {
       console.log('Tout en stock');
     }
   });
   // Sortie: Rupture de stock: PC portable
   ```

3. **Recherche dans les logs d'erreur**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App démarrée' },
     { timestamp: 2, level: 'info' as const, message: 'Utilisateur connecté' },
     { timestamp: 3, level: 'error' as const, message: 'Échec de connexion' },
     { timestamp: 4, level: 'info' as const, message: 'Nouvelle tentative réussie' }
   ] as LogEntry[]);

   // Rechercher la première erreur
   logs$.pipe(
     find(log => log.level === 'error')
   ).subscribe(log => {
     if (log) {
       console.log(`Erreur détectée: ${log.message} (heure: ${log.timestamp})`);
     }
   });
   // Sortie: Erreur détectée: Échec de connexion (heure: 3)
   ```


## 🧠 Exemple de code pratique (Recherche de produit)

Un exemple de recherche de produits correspondant à des conditions spécifiques dans un inventaire.

```ts
import { from, fromEvent } from 'rxjs';
import { find } from 'rxjs';

interface Product {
  id: string;
  name: string;
  price: number;
  category: string;
}

const products: Product[] = [
  { id: 'P1', name: 'Souris sans fil', price: 2980, category: 'Périphériques PC' },
  { id: 'P2', name: 'Clavier mécanique', price: 8980, category: 'Périphériques PC' },
  { id: 'P3', name: 'Clé USB 64GB', price: 1480, category: 'Stockage' },
  { id: 'P4', name: 'Moniteur 27 pouces', price: 29800, category: 'Affichage' },
  { id: 'P5', name: 'Support PC portable', price: 3980, category: 'Périphériques PC' }
];

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Recherche de produit';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Entrer le prix maximum';
input.style.marginRight = '10px';
container.appendChild(input);

const searchButton = document.createElement('button');
searchButton.textContent = 'Rechercher';
container.appendChild(searchButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Traitement de la recherche
fromEvent(searchButton, 'click').subscribe(() => {
  const maxPrice = parseInt(input.value);

  if (isNaN(maxPrice)) {
    result.textContent = 'Veuillez entrer un prix';
    result.style.color = 'red';
    return;
  }

  from(products).pipe(
    find(product => product.price <= maxPrice)
  ).subscribe(product => {
    if (product) {
      result.innerHTML = `
        <strong>Trouvé !</strong><br>
        Nom: ${product.name}<br>
        Prix: ¥${product.price.toLocaleString()}<br>
        Catégorie: ${product.category}
      `;
      result.style.color = 'green';
    } else {
      result.textContent = `Aucun produit trouvé à ¥${maxPrice.toLocaleString()} ou moins`;
      result.style.color = 'orange';
    }
  });
});
```

Ce code recherche et affiche le premier produit dont le prix est inférieur ou égal au prix saisi par l'utilisateur.


## 🎯 Différence avec filter

`find` et `filter` sont utilisés à des fins différentes.

```ts
import { from } from 'rxjs';
import { find, filter } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: émet toutes les valeurs correspondantes
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter terminé')
});
// Sortie: 7, 8, 9, 10, filter terminé

// find: émet uniquement la première valeur correspondante
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('find terminé')
});
// Sortie: 7, find terminé
```

| Opérateur | Nombre de sorties | Moment de terminaison | Cas d'utilisation |
|---|---|---|---|
| `filter(predicate)` | Toutes les valeurs correspondantes | Quand le flux original termine | Filtrage de données |
| `find(predicate)` | Uniquement la première correspondante | Immédiatement à la découverte | Recherche/vérification d'existence |


## 📋 Utilisation type-safe

Un exemple d'implémentation type-safe utilisant les génériques TypeScript.

```ts
import { Observable, from } from 'rxjs';
import { find } from 'rxjs';

interface Task {
  id: number;
  title: string;
  completed: boolean;
  priority: 'high' | 'medium' | 'low';
}

function findTaskById(
  tasks$: Observable<Task>,
  id: number
): Observable<Task | undefined> {
  return tasks$.pipe(
    find(task => task.id === id)
  );
}

function findFirstIncompleteTask(
  tasks$: Observable<Task>
): Observable<Task | undefined> {
  return tasks$.pipe(
    find(task => !task.completed)
  );
}

// Exemple d'utilisation
const tasks$ = from([
  { id: 1, title: 'Tâche A', completed: true, priority: 'high' as const },
  { id: 2, title: 'Tâche B', completed: false, priority: 'medium' as const },
  { id: 3, title: 'Tâche C', completed: false, priority: 'low' as const }
] as Task[]);

// Recherche par ID
findTaskById(tasks$, 2).subscribe(task => {
  if (task) {
    console.log(`Trouvé: ${task.title}`);
  } else {
    console.log('Tâche non trouvée');
  }
});
// Sortie: Trouvé: Tâche B

// Recherche de tâche non terminée
findFirstIncompleteTask(tasks$).subscribe(task => {
  if (task) {
    console.log(`Prochaine tâche: ${task.title} (priorité: ${task.priority})`);
  }
});
// Sortie: Prochaine tâche: Tâche B (priorité: medium)
```


## 🔄 Différence entre find et findIndex

RxJS dispose également de l'opérateur `findIndex`.

```ts
import { from } from 'rxjs';
import { find, findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// find: retourne la valeur
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Sortie: 30

// findIndex: retourne l'index
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Sortie: 2 (index de 30)
```

| Opérateur | Valeur retournée | Si non trouvé |
|---|---|---|
| `find(predicate)` | La valeur elle-même | `undefined` |
| `findIndex(predicate)` | L'index (nombre) | `-1` |


## ⚠️ Erreurs courantes

> [!NOTE]
> `find` émet `undefined` si aucune valeur n'est trouvée. Ce n'est pas une erreur. Si vous avez besoin d'une erreur, utilisez `first`.

### Incorrect : Attendre une gestion d'erreur quand non trouvé

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ❌ Mauvais exemple: la gestion d'erreur attendue n'est pas appelée
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,
  error: err => console.log('Erreur:', err) // Non appelé
});
// Sortie: undefined
```

### Correct : Vérifier undefined ou utiliser first

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ✅ Bon exemple 1: vérifier undefined
numbers$.pipe(
  find(n => n > 10)
).subscribe(result => {
  if (result !== undefined) {
    console.log('Trouvé:', result);
  } else {
    console.log('Non trouvé');
  }
});
// Sortie: Non trouvé

// ✅ Bon exemple 2: utiliser first si une erreur est nécessaire
numbers$.pipe(
  first(n => n > 10, 0) // Spécifier une valeur par défaut
).subscribe({
  next: console.log,
  error: err => console.log('Erreur:', err.message)
});
// Sortie: 0
```


## 🎓 Résumé

### Quand utiliser find
- ✅ Quand vous voulez trouver la première valeur satisfaisant une condition
- ✅ Quand vous voulez vérifier l'existence d'une valeur
- ✅ Quand vous voulez gérer le cas non trouvé avec `undefined`
- ✅ Quand vous cherchez un élément spécifique dans un tableau ou une liste

### Quand utiliser first
- ✅ Quand vous voulez récupérer la première valeur
- ✅ Quand vous voulez émettre une erreur si non trouvé

### Quand utiliser filter
- ✅ Quand vous avez besoin de toutes les valeurs correspondantes
- ✅ Quand l'objectif est le filtrage de données

### Points d'attention
- ⚠️ `find` émet `undefined` si non trouvé (pas une erreur)
- ⚠️ Termine immédiatement à la première valeur correspondante
- ⚠️ En TypeScript, le type de retour est `T | undefined`


## 🚀 Prochaines étapes

- **[first](./first)** - Apprendre à récupérer la première valeur
- **[filter](./filter)** - Apprendre le filtrage basé sur les conditions
- **[findIndex](./findIndex)** - Apprendre à récupérer l'index de la première valeur correspondante
- **[Exemples pratiques d'opérateurs de filtrage](./practical-use-cases)** - Apprendre des cas d'utilisation réels
