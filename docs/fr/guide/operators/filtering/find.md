---
description: "find est un opérateur de filtrage RxJS qui trouve la première valeur qui satisfait une condition et la sort, complétant ainsi le flux immédiatement. Il est idéal pour les situations où vous souhaitez trouver un élément spécifique dans un tableau ou une liste, comme la recherche d'utilisateurs, la vérification de l'inventaire ou la détection des journaux d'erreurs. Si aucune valeur n'est trouvée, la sortie est indéfinie et, en TypeScript, la valeur de retour est de type T | indéfini."
---

# find - trouver la première valeur qui satisfait la condition

L'opérateur `find` trouve et affiche la **première valeur qui satisfait la condition** et termine le flux immédiatement. Si aucune valeur n'est trouvée, il affiche `undefined`.

## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Sortie.: 8(premier nombre pair)
```

**Flux d'opérations** :.
1. vérifier 1, 3, 5, 7 → condition non remplie
2. contrôle 8 → condition remplie → sortie 8 et complète
3. 9, 10 non évalués

[🌐 Official RxJS documentation - `find`](https://rxjs.dev/api/operators/find)

## 🆚 Contraste avec first

`find` et `first` sont similaires, mais leur utilisation est différente.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: Première valeur satisfaisant la condition (la condition est facultative)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Sortie.: 7

// find: Première valeur satisfaisant la condition (la condition est obligatoire)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Sortie.: 7
```

| Opérateur. | Spécification de la condition | Si aucune valeur n'est trouvée | Cas d'utilisation. |
|---|---|---|---|
| `premier()` | Option | Erreur (`EmptyError`) | Obtenir la première valeur |
| `first(predicate)` | Option | Erreur (`EmptyError`) | Obtention conditionnelle. |
| `find(predicate)` | Obligatoire. | Sortie `undefined`. | Recherche et vérification de l'existence |

## 💡 Modèle d'utilisation typique

1. **Recherche d'un utilisateur**.

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

   // ID(la condition est facultative)2Recherche d'utilisateurs ayant
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Trouvé: ${user.name}`);
     } else {
       console.log('Utilisateur non trouvé');
     }
   });
   // Sortie.: Trouvé: Bob
   ```

2. **Vérification de l'inventaire**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'Ordinateur portablePC', stock: 0 },
     { id: 'A2', name: 'Souris', stock: 15 },
     { id: 'A3', name: 'Claviers', stock: 8 }
   ] as Product[]);

   // Voir ce qui est en rupture de stock
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`En rupture de stock: ${product.name}`);
     } else {
       console.log('Tous en stock');
     }
   });
   // Sortie.: En rupture de stock: Ordinateur portablePC
   ```

3. **Rechercher le journal des erreurs**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 4, level: 'info' as const, message: 'Retry successful' }
   ] as LogEntry[]);

   // Rechercher la première erreur
   logs$.pipe(
     find(log => log.level === 'error')
   ).subscribe(log => {
     if (log) {
       console.log(`Détection d'erreur: ${log.message} (Temps: ${log.timestamp})`);
     }
   });
   // Sortie.: Détection d'erreur: Connection failed (Temps: 3)
   ```

## 🧠 Exemple de code pratique (recherche de produits)

Il s'agit d'un exemple de recherche de produits correspondant à des critères spécifiques dans le stock.

```

ts.
import { from, fromEvent } from 'rxjs' ;
import { find } from 'rxjs' ;

interface Product {
  id : string ;
  name : chaîne de caractères ;
  price : nombre ;
  category : string ;
}

const products : Product[] = [
  { id : 'P1', name : 'Wireless mouse', price : 2980, category : 'PC peripherals' }
  { id : 'P2', name : 'Mechanical Keyboard', price : 8980, category : 'PC Peripherals' }
  { id : 'P3', name : 'Clé USB 64GB', price : 1480, category : 'Storage' }
  { id : 'P4', name : 'Moniteur 27 pouces', price : 29800, category : 'Displays' }
  { id : 'P5', name : 'Support d'ordinateur portable', price : 3980, category : 'Périphériques PC' }
] ;

// Création d'éléments d'interface utilisateur
const container = document.createElement('div') ;.
document.body.appendChild(container) ;

const title = document.createElement('h3') ;
title.textContent = "Recherche de produits" ;
container.appendChild(title) ;

const input = document.createElement('input') ;
input.type = 'number' ;
input.placeholder = "Entrez le prix maximum" ;
input.style.marginRight = '10px' ;
container.appendChild(input) ;

const searchButton = document.createElement('button') ;
searchButton.textContent = 'search' ;
container.appendChild(searchButton) ;

const result = document.createElement('div') ;
result.style.marginTop = '10px' ;
container.appendChild(result) ;

// Traitement de la recherche
// Note : à l'origine, le schéma recommandé est d'aplatir avec un switchMap, mais,
// Note : Bien que le pattern recommandé soit d'aplatir avec un switchMap, // ici nous imbriquons le subscribe pour la lisibilité, // parce qu'il inclut la validation de l'IU (retour anticipé).
// Considérez une implémentation plate utilisant `switchMap` dans le code de production.
fromEvent(searchButton, 'click').subscribe(() => {
  const maxPrice = parseInt(input.value) ;.

  if (isNaN(maxPrice)) {
    result.textContent = 'Veuillez saisir un prix' ;
    result.style.colour = 'red' ;
    return ;
  }

  // Nest subscribe : recommandé à l'origine pour aplatir avec switchMap
  from(produits).pipe(
    find(produit => produit.prix <= maxPrice)
  ).subscribe(produit => {
    if (product) {
      result.innerHTML = `
        <strong>Trouvé ! </strong><br>
        Nom du produit : ${product.name}<br>
        Prix : ${product.price.toLocaleString()}<br>
        Catégorie : ${product.category}
      ` ;
      result.style.color = 'green' ;
    } else {
      result.textContent = `¥${maxPrice.toLocaleString()} ou moins produit non trouvé ` ;
      result.style.color = 'orange' ; }
    }
  }) ;
}) ;

```

Ce code recherche et affiche le premier produit dont le prix est inférieur à celui introduit par l'utilisateur.

## 🎯 filter La différence entre

`find` et `filter` sont utilisées à des fins différentes.

```

ts.
import { from } from 'rxjs' ;
import { find, filter } from 'rxjs' ;

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]) ;

// filter : sort toutes les valeurs qui correspondent à la condition
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next : console.log, .
  complete : () => console.log('filter complete')
}) ;
// Sortie : 7, 8, 9, 10, filtre terminé

// find : sortie uniquement de la première valeur qui correspond à la condition
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next : console.log, .
  complete : () => console.log('find complete')
}) ;
// résultat : 7, find complete

```

| Opérateur | Nombre de sorties | Délai d'exécution | Cas d'utilisation |
|---|---|---|---|
| `filter(predicate)` | Toutes les valeurs correspondant à la condition | À la fin du flux original | Affinage des données |
| `find(predicate)` | Seule la première valeur correspondant aux critères | Immédiatement après la découverte | Recherche et vérification de l'existence |

## 📋 Utilisation à sécurité de type

TypeScript Il s'agit d'un exemple d'implémentation à sécurité de type qui utilise les éléments génériques dans le cadre de l'analyse des données.

```

ts.
import { Observable, from } from 'rxjs' ;
import { find } from 'rxjs' ;

interface Task {
  id : nombre ;
  title : chaîne de caractères ;
  complete : booléen ;
  priority : 'high' | 'medium' | 'low' ; }
}

function findTaskById(
  tasks$ : Observable,.
  id : nombre
) : Observable | undefined> {
  return tasks$.pipe(
    find(task => task.id === id)
  ) ;
}

function findFirstIncompleteTask(
  tasks$ : Observable
) : Observable | undefined> {
  return tasks$.pipe(
    find(task => !task.complete)
  ) ;
}

// Exemple d'utilisation
const tasks$ = from([.
  { id : 1, title : 'Tâche A', complete : true, priority : 'high' as const }
  { id : 2, title : 'Tâche B', complete : false, priority : 'medium' as const }
  { id : 3, title : 'Tâche C', complete : false, priority : 'low' as const }
] en tant que Task[]) ;.

// Recherche par ID
findTaskById(tasks$, 2).subscribe(task => {
  if (task) {
    console.log(`trouvé : ${task.title}`) ;
  } else {
    console.log('Task not found') ; }
  }
}) ;
// Sortie : trouvée : tâche B

// Recherche des tâches inachevées
findFirstIncompleteTask(tasks$).subscribe(task => {
  if (task) {
    console.log(`Tâche suivante : ${task.title} (priorité : ${task.priority})`) ;
  }
}) ;
// Sortie : tâche suivante : tâche B (priorité : moyenne)

```

## 🔄 find et findIndex La différence entre

RxJSdans les `findIndex` sont également disponibles.

```

ts
import { from } from 'rxjs' ;
import { find, findIndex } from 'rxjs' ;

const numbers$ = from([10, 20, 30, 40, 50]) ;

// find : renvoie une valeur
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log) ;.
// sortie : 30

// findIndex : return index
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log) ;.
// Résultat : 2 (indice de 30)

```

| Opérateur | Retourner la valeur | si la valeur n'est pas trouvée |
|---|---|---|
| `find(predicate)` | Valeur elle-même | `undefined` |
| `findIndex(predicate)` | Index (valeur numérique) | `-1` |

## ⚠️ Une erreur fréquente

> [!NOTE]
> `find` si la valeur n'est pas trouvée. `undefined` est affiché. Cela n'entraîne pas d'erreur. Si une erreur est nécessaire, il faut utiliser `first` pour être utilisée.

### Erreur.: Traitement de l'erreur attendue si la valeur n'est pas trouvée.

```

ts.
import { from } from 'rxjs' ;
import { find } from 'rxjs' ;

const numbers$ = from([1, 3, 5, 7]) ;

// ❌ Mauvais exemple : la gestion des erreurs est attendue mais n'est pas appelée
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next : console.log, .
  error : err => console.log('Error:', err) // non appelé
}) ;
// résultat : non défini

```

### Positif: undefined Vérifier ou first utiliser le

```

ts.
import { from } from 'rxjs' ;
import { find, first } from 'rxjs' ;

const numbers$ = from([1, 3, 5, 7]) ;

// ✅ Bon exemple 1 : vérification de l'absence de définition
numbers$.pipe(
  find(n => n > 10)
).subscribe(result => {
  if (result ! == undefined) {
    console.log('Found:', result) ;
  } else {
    console.log('Not found:') ; }
  }
}) ;
// Résultat : non trouvé

// ✅ Bon exemple 2 : utilisez le first si vous avez besoin d'une erreur
numbers$.pipe(
  first(n => n > 10, 0) // spécifie la valeur par défaut
).subscribe({
  next : console.log,.
  error : err => console.log('Error:', err.message)
}) ;
// Sortie : 0
```

## 🎓 Résumé

### Quand utiliser find.
- ✅ Si vous voulez trouver la première valeur qui satisfait une condition.
- ✅ Lorsque vous voulez vérifier l'existence d'une valeur
- ✅ Lorsque vous voulez traiter une valeur comme `undefined` si elle n'est pas trouvée.
- ✅ Lorsque vous voulez trouver un élément spécifique dans un tableau ou une liste

### Quand vous devez utiliser first
- ✅ Si vous voulez obtenir la première valeur
- ✅ Si vous voulez afficher une erreur si la valeur n'est pas trouvée

### Quand faut-il utiliser le filtre ?
- ✅ Si vous avez besoin de toutes les valeurs correspondant à une condition
- ✅ Si vous voulez filtrer les données

### Notes.
- ⚠️ `find` produit `undefined` s'il n'est pas trouvé (ce n'est pas une erreur)
- ⚠️ Complète immédiatement avec la première valeur qui satisfait la condition
- ⚠️ TypeScript donne une valeur de retour de type `T | undefined`.

## 🚀 Prochaine étape.

- **[first](. /first)** - apprendre à obtenir la première valeur.
- **[filter](. /filter)** - Apprenez à filtrer sur la base de conditions.
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - apprendre à obtenir l'index de la première valeur qui satisfait une condition (documentation officielle).
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - apprendre des cas d'utilisation réels
