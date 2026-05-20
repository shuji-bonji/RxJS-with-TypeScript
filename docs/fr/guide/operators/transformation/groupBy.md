---
description: "L'opérateur groupBy regroupe les valeurs de flux en fonction d'une clé spécifiée et crée un Observable distinct pour chaque groupe. Mise en œuvre à sécurité de type et cas d'utilisation pratiques en TypeScript, notamment l'agrégation par catégorie, le traitement par utilisateur et la catégorisation des données."
---

# groupBy - regrouper les valeurs en fonction d'une clé

L'opérateur `groupBy` **groupe** les valeurs issues d'un flux en fonction d'une clé spécifiée et produit chaque groupe sous la forme d'un Observable séparé.
Ceci est utile si vous voulez catégoriser des données ou appliquer un traitement différent à chaque groupe.

## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

interface Person {
  name: string;
  age: number;
}

const people: Person[] = [
  { name: 'Taro', age: 25 },
  { name: 'Hanako', age: 30 },
  { name: 'Jiro', age: 25 },
  { name: 'Misaki', age: 30 },
  { name: 'Kenta', age: 35 },
];

from(people).pipe(
  groupBy(person => person.age), // Regroupés par âge
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(arr => ({ age: group.key, people: arr }))
    )
  )
).subscribe(result => {
  console.log(`Âge ${result.age}:`, result.people);
});

// Sortie:
// Âge 25: [{name: 'Taro', age: 25}, {name: 'Jiro', age: 25}]
// Âge 30: [{name: 'Hanako', age: 30}, {name: 'Misaki', age: 30}]
// Âge 35: [{name: 'Kenta', age: 35}]
```

- `groupBy(person => person.age)` pour grouper par l'âge comme clé.
- Chaque groupe est traité comme un `GroupedObservable` et la clé du groupe est accessible via la propriété `key`.
- Traiter chaque Observable groupé avec `mergeMap`.

[🌐 Official RxJS documentation - `groupBy`](https://rxjs.dev/api/operators/groupBy)

## 💡 Modes d'utilisation typiques

- Catégorisation des données par catégorie
- Traitement agrégé par groupe
- Traitement des journaux et des événements par type
- Regroupement et transformation des données

## 🧠 Exemples de code pratique (avec interface utilisateur)

Cet exemple montre le nombre de pièces regroupées par couleur lorsqu'un bouton est cliqué.

```ts
import { fromEvent, from } from 'rxjs';
import { groupBy, mergeMap, toArray, switchMap, map } from 'rxjs';

// Bouton de création
const colors = ['Rouge', 'bleu', 'Vert', 'Jaune'];
colors.forEach(color => {
  const button = document.createElement('button');
  button.textContent = color;
  button.style.margin = '5px';
  button.style.padding = '10px';
  button.dataset.color = color;
  document.body.appendChild(button);
});

const calculateButton = document.createElement('button');
calculateButton.textContent = 'Agréger.';
calculateButton.style.margin = '5px';
calculateButton.style.padding = '10px';
document.body.appendChild(calculateButton);

// Créer une zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Enregistrer les couleurs cliquées
const clicks: string[] = [];

// Événements de clics sur les boutons de couleur
fromEvent(document, 'click').subscribe((event: Event) => {
  const target = event.target as HTMLElement;
  const color = target.dataset.color;
  if (color) {
    clicks.push(color);
    output.innerHTML = `Couleur sélectionnée: ${clicks.join(', ')}`;
  }
});

// Regroupé lors du clic sur le bouton d'agrégation
fromEvent(calculateButton, 'click').pipe(
  switchMap(() =>
    from(clicks).pipe(
      groupBy(color => color),
      mergeMap(group =>
        group.pipe(
          toArray(),
          map(items => ({ color: group.key, count: items.length }))
        )
      ),
      toArray()
    )
  )
).subscribe(results => {
  if (results.length === 0) {
    output.innerHTML = '<p>Aucune couleur n'a encore été sélectionnée</p>';
    return;
  }
  const resultText = results
    .map(r => `${r.color}: ${r.count}Temps`)
    .join('<br>');
  output.innerHTML = `<h3>Résultat de l'agrégation</h3>${resultText}`;
});
```

- Cliquez sur le bouton Couleur pour sélectionner une couleur
- Grouper par couleur avec le bouton `totalise` et afficher le nombre de pièces.
- Regroupez par couleur avec le bouton `groupBy` et comptez le nombre d'éléments dans chaque groupe.

## 🎯 Exemple de totalisation par catégorie

Voici un exemple de regroupement de produits par catégorie et de calcul du montant total pour chaque catégorie.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, reduce, map } from 'rxjs';

interface Product {
  name: string;
  category: string;
  price: number;
}

const products: Product[] = [
  { name: 'Pommes', category: 'Fruits', price: 150 },
  { name: 'Mandarines', category: 'Fruits', price: 100 },
  { name: 'Carottes', category: 'Légumes', price: 80 },
  { name: 'Tomates', category: 'Légumes', price: 120 },
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
  console.log(`${result.category}: ${result.total}Cercle`);
});

// Sortie:
// Fruits: 250Cercle
// Légumes: 200Cercle
// Produits laitiers: 500Cercle
```

## 🎯 Exemple d'utilisation d'un sélecteur d'élément.

Lors du regroupement, les valeurs peuvent également être converties.

```ts
import { from } from 'rxjs';
import { groupBy, map, mergeMap, toArray } from 'rxjs';

interface Student {
  name: string;
  grade: number;
  score: number;
}

const students: Student[] = [
  { name: 'Taro', grade: 1, score: 85 },
  { name: 'Hanako', grade: 2, score: 92 },
  { name: 'Jiro', grade: 1, score: 78 },
  { name: 'Misaki', grade: 2, score: 88 },
];

from(students).pipe(
  groupBy(
    student => student.grade,           // Sélecteur de clé
    student => student.name             // Sélecteur d'éléments (ne contient que des noms)
  ),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(names => ({ grade: group.key, students: names }))
    )
  )
).subscribe(result => {
  console.log(`${result.grade}Année étudiante:`, result.students.join(', '));
});

// Sortie:
// 1Année étudiante: Taro, Jiro
// 2Année étudiante: Hanako, Misaki
```

- 1er argument : sélecteur de clé (critères de regroupement)
- Deuxième argument : sélecteur d'éléments (valeurs à stocker dans le groupe).

## 🎯 Utilisation d'un groupBy à sécurité de type

Il s'agit d'un exemple d'utilisation de l'inférence de type TypeScript.

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
  { level: 'info', message: 'Démarrage de l'application', timestamp: 1000 },
  { level: 'warning', message: 'Message d'avertissement', timestamp: 2000 },
  { level: 'error', message: 'Une erreur s'est produite', timestamp: 3000 },
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
  console.log(`[${result.level.toUpperCase()}] ${result.count}Cas`);
  result.messages.forEach(msg => console.log(`  - ${msg}`));
});

// Sortie:
// [INFO] 2Cas
//   - Démarrage de l'application
//   - Traitement terminé
// [WARNING] 1Cas
//   - Message d'avertissement
// [ERROR] 2Cas
//   - Une erreur s'est produite
//   - Erreur de connexion
```

## 🎯 Appliquer différents processus à différents groupes

Il s'agit d'un exemple d'application de traitements différents à chaque groupe.

```ts
import { from, of } from 'rxjs';
import { groupBy, mergeMap, delay, map } from 'rxjs';

interface Task {
  id: number;
  priority: 'high' | 'medium' | 'low';
  name: string;
}

const tasks: Task[] = [
  { id: 1, priority: 'high', name: 'Tâche urgente' },
  { id: 2, priority: 'low', name: 'Tâche reportée' },
  { id: 3, priority: 'high', name: 'Tâches importantes' },
  { id: 4, priority: 'medium', name: 'Tâches normales' },
];

from(tasks).pipe(
  groupBy(task => task.priority),
  mergeMap(group => {
    // Les délais sont fixés en fonction de la priorité
    const delayTime =
      group.key === 'high' ? 0 :
      group.key === 'medium' ? 1000 :
      2000;

    return group.pipe(
      delay(delayTime),
      map(task => ({ ...task, processedAt: Date.now() }))
    );
  })
).subscribe(task => {
  console.log(`[${task.priority}] ${task.name} Traitement`);
});

// Sortie (par ordre de priorité):
// [high] Tâche urgente Traitement
// [high] Tâches importantes Traitement
// (1(après 1,5 seconde)
// [medium] Tâches normales Traitement
// (plus loin)1(après 1,5 seconde)
// [low] Tâche reportée Traitement
```

## ⚠️ Notes.

### Gestion des abonnements pour le groupe Observable.

`groupBy` crée un Observable pour chaque groupe. Ces Observable peuvent provoquer des fuites de mémoire s'ils ne sont pas correctement souscrits (subscribe).

```ts
// ❌ Mauvais exemple: Le groupeObservablene s'abonne pas à
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd')
).subscribe(group => {
  // Le groupeObservablePas d'abonnement
  console.log('Le groupe:', group.key);
});
```

**Mesures** : toujours traiter chaque groupe avec `mergeMap`, `concatMap`, `switchMap`, etc.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

// ✅ Bon exemple: Chaque groupe est traité de manière appropriée
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd'),
  mergeMap(group =>
    group.pipe(toArray())
  )
).subscribe(console.log);
```

### Génération dynamique de groupes

Un nouvel Observable de groupe est créé à chaque fois qu'une nouvelle clé apparaît. Il faut faire attention s'il y a beaucoup de types de clés.

```ts
// Exemple d'un nombre potentiellement infini de types de clés
fromEvent(document, 'click').pipe(
  groupBy(() => Math.random()) // Clés différentes à chaque fois
).subscribe(); // Risque de fuites de mémoire
```

## 📚 Opérateurs associés.

- [`partition`](https://rxjs.dev/api/index/function/partition) - divise en deux Observable par condition.
- [`reduce`](./reduce) - Pour obtenir le résultat final de l'agrégat.
- [`scan`](./scan) - Agrégation cumulative.
- [`toArray`](../utility/toArray) - Combine toutes les valeurs dans un tableau.

## Résumé.

L'opérateur `groupBy` peut regrouper des valeurs dans un flux basé sur des clés et **traiter chaque groupe comme un Observable** séparé. Ceci est très utile pour le traitement de données complexes, comme la catégorisation des données, l'agrégation par catégorie et le traitement de chaque groupe différemment. Cependant, chaque Observable de groupe doit être correctement souscrit et est habituellement utilisé en conjonction avec un `mergeMap` ou similaire.
