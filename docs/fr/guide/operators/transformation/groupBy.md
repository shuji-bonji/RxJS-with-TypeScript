---
description: "L'opérateur groupBy regroupe les valeurs d'un flux en fonction d'une clé spécifiée et crée des Observables distincts pour chaque groupe. Explique l'implémentation TypeScript sécurisée et les cas d'utilisation pratiques pour l'agrégation par catégorie, le traitement spécifique par utilisateur et la classification des données."
---

# groupBy - Regrouper des valeurs en fonction d'une clé

L'opérateur `groupBy` **groupe les valeurs émises d'un flux en fonction d'une clé spécifiée** et produit chaque groupe comme un Observable individuel.
Pratique pour classer les données par catégorie ou appliquer un traitement différent à chaque groupe.

## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

interface Person {
  name: string;
  age: number;
}

const people: Person[] = [
  { name: 'Pierre', age: 25 },
  { name: 'Marie', age: 30 },
  { name: 'Jean', age: 25 },
  { name: 'Sophie', age: 30 },
  { name: 'Paul', age: 35 },
];

from(people).pipe(
  groupBy(person => person.age), // Grouper par âge
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(arr => ({ age: group.key, people: arr }))
    )
  )
).subscribe(result => {
  console.log(`Âge ${result.age} :`, result.people);
});

// Sortie :
// Âge 25 : [{name: 'Pierre', age: 25}, {name: 'Jean', age: 25}]
// Âge 30 : [{name: 'Marie', age: 30}, {name: 'Sophie', age: 30}]
// Âge 35 : [{name: 'Paul', age: 35}]
```

- `groupBy(person => person.age)` groupe par âge comme clé
- Chaque groupe est traité comme `GroupedObservable`, et la clé du groupe est accessible via la propriété `key`
- Traiter chaque Observable de groupe avec `mergeMap`

[🌐 Documentation officielle RxJS - groupBy](https://rxjs.dev/api/operators/groupBy)

## 💡 Modes d'utilisation typiques

- Classification des données par catégorie
- Traitement d'agrégation par groupe
- Traitement spécifique au type des logs ou événements
- Regroupement et transformation des données

## 🎯 Exemple d'agrégation par catégorie

Exemple classant des produits par catégorie et calculant le montant total par catégorie.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, reduce, map } from 'rxjs';

interface Product {
  name: string;
  category: string;
  price: number;
}

const products: Product[] = [
  { name: 'Pomme', category: 'Fruits', price: 150 },
  { name: 'Orange', category: 'Fruits', price: 100 },
  { name: 'Carotte', category: 'Légumes', price: 80 },
  { name: 'Tomate', category: 'Légumes', price: 120 },
  { name: 'Lait', category: 'Produits laitiers', price: 200 },
  { name: 'Fromage', category: 'Produits laitiers', price: 300 },
];

from(products).pipe(
  groupBy(product => product.category),
  mergeMap(group =>
    group.pipe(
      reduce((total, product) => total + product.price, 0),
      map(total => ({ category: group.key, total }))
    )
  )
).subscribe(result => {
  console.log(`${result.category} : ${result.total}€`);
});

// Sortie :
// Fruits : 250€
// Légumes : 200€
// Produits laitiers : 500€
```

## 🎯 Exemple de groupBy type-safe

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

type LogLevel = 'info' | 'warning' | 'error';

interface LogEntry {
  level: LogLevel;
  message: string;
  timestamp: number;
}

const logs: LogEntry[] = [
  { level: 'info', message: 'Application démarrée', timestamp: 1000 },
  { level: 'warning', message: 'Message d\'avertissement', timestamp: 2000 },
  { level: 'error', message: 'Erreur survenue', timestamp: 3000 },
  { level: 'info', message: 'Traitement terminé', timestamp: 4000 },
  { level: 'error', message: 'Erreur de connexion', timestamp: 5000 },
];

from(logs).pipe(
  groupBy(log => log.level),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(entries => ({
        level: group.key,
        count: entries.length,
        messages: entries.map(e => e.message)
      }))
    )
  )
).subscribe(result => {
  console.log(`[${result.level.toUpperCase()}] ${result.count} entrée(s)`);
  result.messages.forEach(msg => console.log(`  - ${msg}`));
});

// Sortie :
// [INFO] 2 entrée(s)
//   - Application démarrée
//   - Traitement terminé
// [WARNING] 1 entrée(s)
//   - Message d'avertissement
// [ERROR] 2 entrée(s)
//   - Erreur survenue
//   - Erreur de connexion
```

## ⚠️ Notes importantes

### Gestion des abonnements aux Observables de groupe

`groupBy` crée un Observable pour chaque groupe. Ces Observables peuvent provoquer des fuites de mémoire s'ils ne sont pas correctement souscrits.

```ts
// ❌ Mauvais exemple : ne pas souscrire aux Observables de groupe
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'pair' : 'impair')
).subscribe(group => {
  // L'Observable de groupe n'est pas souscrit
  console.log('Groupe :', group.key);
});
```

**Solution** : Toujours traiter chaque groupe avec `mergeMap`, `concatMap`, `switchMap`, etc.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

// ✅ Bon exemple : traiter correctement chaque groupe
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'pair' : 'impair'),
  mergeMap(group =>
    group.pipe(toArray())
  )
).subscribe(console.log);
```

### Création dynamique de groupes

Un nouvel Observable de groupe est créé chaque fois qu'une nouvelle clé apparaît. Attention si le nombre de types de clés est important.

## 📚 Opérateurs associés

- [`partition`](https://rxjs.dev/api/index/function/partition) - Diviser en deux Observables par condition
- [`reduce`](./reduce) - Obtenir le résultat final de l'agrégation
- [`scan`](./scan) - Agrégation cumulative
- [`toArray`](/fr/guide/operators/utility/toArray) - Rassembler toutes les valeurs dans un tableau

## Résumé

L'opérateur `groupBy` groupe les valeurs d'un flux en fonction d'une clé et permet de **traiter chaque groupe comme un Observable individuel**. Très utile pour le traitement de données complexes comme la classification des données, l'agrégation par catégorie et le traitement différent pour chaque groupe.
