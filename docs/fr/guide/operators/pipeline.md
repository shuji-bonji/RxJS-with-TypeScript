---
description: "Un regard approfondi sur les bases et les applications de la construction de pipelines en utilisant la méthode pipe() dans RxJS. Vous apprendrez le chaînage d'opérateurs, l'utilisation de l'inférence de type TypeScript, la création d'opérateurs personnalisés, les techniques de débogage et l'optimisation des performances à l'aide d'exemples de code pratiques. Il introduit également les principes fondamentaux de la programmation fonctionnelle."
---

# Qu'est-ce que le pipeline RxJS ?

Le pipeline dans RxJS est un mécanisme permettant d'appliquer une série d'opérations (opérateurs) à un Observable de manière séquentielle. Le pipeline vous permet de transformer, de filtrer et de combiner des flux de données en plusieurs étapes et de contrôler le flux de données dans un style de programmation déclaratif.

## Structure de base d'un pipeline

[📘 RxJS officiel : pipe()](https://rxjs.dev/api/index/function/pipe)

La méthode RxJS `pipe()` est utilisée pour construire un pipeline. La syntaxe est la suivante.

```ts
import { Observable } from 'rxjs';
import { map, filter, tap } from 'rxjs';

const source$: Observable<number> = // un Observable quelconque
source$.pipe(
  // Chaîner plusieurs opérateurs
  operator1(),
  operator2(),
  operator3(),
  // ...
).subscribe(value => {
  // Traiter le résultat
});
```

## Exemples pratiques

### Conversion de données de base

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// Flux de nombres
const numbers$ = of(1, 2, 3, 4, 5);

// Construire un pipeline
numbers$.pipe(
  // Ne laisser passer que les nombres pairs
  filter(n => n % 2 === 0),
  // Doubler la valeur
  map(n => n * 2)
).subscribe(
  value => console.log(`Résultat : ${value}`)
);

// Sortie :
// Résultat : 4
// Résultat : 8
```

### Traitement de données complexes

```ts
import { fromEvent, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
};
type Post = {
  userId: number;
  id: number;
  title: string;
  body: string;
};

// Créer un élément DOM
const searchButton = document.createElement('button');
searchButton.innerText = 'Rechercher';
document.body.appendChild(searchButton);

const resultBox = document.createElement('div');
resultBox.id = 'results';
document.body.appendChild(resultBox);

// Requête API au clic du bouton
fromEvent(searchButton, 'click')
  .pipe(
    switchMap(() =>
      // Premier appel API
      ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
        // Deuxième appel API pour récupérer les publications de l'utilisateur
        switchMap((user) => {
          const header = document.createElement('h3');
          header.textContent = `Utilisateur : ${user.name}`;
          resultBox.innerHTML = ''; // Effacer les résultats précédents
          resultBox.appendChild(header);

          return ajax.getJSON<Post[]>(
            `https://jsonplaceholder.typicode.com/posts?userId=${user.id}`
          );
        }),
        // Récupérer uniquement les 3 premières publications
        map((posts) => posts.slice(0, 3))
      )
    )
  )
  .subscribe((posts) => {
    // Processus d'affichage des publications à l'écran
    resultBox.innerHTML += '<h4>Liste des publications :</h4>';
    posts.forEach((post) => {
      const div = document.createElement('div');
      div.innerHTML = `<strong>${post.title}</strong><p>${post.body}</p>`;
      resultBox.appendChild(div);
    });
  });

```


## Avantages du pipeline

Tout d'abord, regardons le code écrit de manière impérative. Comme le montre la suite, le pipeline RxJS permet de le réécrire sous une forme plus lisible et plus facile à maintenir, tout en rendant l'intention du processus claire.

### 1. Lisibilité et maintenabilité améliorées

```ts
// Traitement en style impératif
const data = [
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
];

const activeItems = [];
for (const item of data) {
  if (item.active) {
    activeItems.push({ ...item, label: `Item #${item.id}` });
  }
}
activeItems.sort((a, b) => a.id - b.id);

const div1 = document.createElement('div');
div1.innerHTML = '<h3>Style impératif</h3>';
activeItems.forEach(item => {
  const p = document.createElement('p');
  p.textContent = item.label;
  div1.appendChild(p);
});
document.body.appendChild(div1);
```
⬇️⬇️⬇️
```ts
import { of } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>Lisibilité et maintenabilité améliorées</h3>';
document.body.appendChild(output);

of(
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
).pipe(
  filter(item => item.active),
  map(item => ({ ...item, label: `Item #${item.id}` })),
  toArray(),
  map(array => array.sort((a, b) => a.id - b.id))
).subscribe(sorted => {
  sorted.forEach(item => {
    const div = document.createElement('div');
    div.textContent = item.label;
    output.appendChild(div);
  });
});
```

Le pipeline rend le flux de données clair et élimine la nécessité de réaffecter les variables et de gérer les états intermédiaires.



Le code procédural ci-dessus peut également être réécrit de manière concise dans un style déclaratif en utilisant le pipeline RxJS. En voici un exemple ci-dessous.

### 2. Style de programmation déclaratif

Les pipelines favorisent un style déclaratif qui indique explicitement « ce qu'il faut faire ». L'intention du code est ainsi plus claire.

```ts
// Traitement en style procédural
const usersList = [
  { status: 'active', firstName: 'Taro', lastName: 'Yamada', email: 'taro@example.com' },
  { status: 'inactive', firstName: 'Hanako', lastName: 'Yamada', email: 'hanako@example.com' },
  { status: 'active', firstName: 'John', lastName: 'Doe', email: 'john@example.com' }
];

const activeUsers2 = [];
for (const user of usersList) {
  if (user.status === 'active') {
    const name = `${user.firstName} ${user.lastName}`;
    activeUsers2.push({ name, email: user.email });
  }
}

const div2 = document.createElement('div');
div2.innerHTML = '<h3>Style procédural</h3>';
activeUsers2.forEach(user => {
  const p = document.createElement('p');
  p.textContent = `${user.name} (${user.email})`;
  div2.appendChild(p);
});
document.body.appendChild(div2);
```

⬇️⬇️⬇️

```ts
// Style de programmation déclaratif
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

const out2 = document.createElement('div');
out2.innerHTML = '<h3>Style déclaratif</h3>';
document.body.appendChild(out2);

const users = [
  { status: 'active', firstName: 'Taro', lastName: 'Yamada', email: 'taro@example.com' },
  { status: 'inactive', firstName: 'Hanako', lastName: 'Yamada', email: 'hanako@example.com' },
  { status: 'active', firstName: 'John', lastName: 'Doe', email: 'john@example.com' }
];

from(users).pipe(
  filter(user => user.status === 'active'),
  map(user => ({
    name: `${user.firstName} ${user.lastName}`,
    email: user.email
  }))
).subscribe(user => {
  const div = document.createElement('div');
  div.textContent = `${user.name} (${user.email})`;
  out2.appendChild(div);
});
```


De la même manière, le code qui décrit un processus de manière procédurale peut être réorganisé dans un pipeline. Les processus complexes peuvent être simplement construits en composant des opérateurs individuels.

### 3. Composabilité

Le pipeline permet de construire des processus complexes en combinant de petites opérations.

```ts
// Traitement en style procédural (impératif)
const rawUsers = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const activeUsers = [];
for (const user of rawUsers) {
  if (user.status === 'active') {
    const fullName = `${user.firstName} ${user.lastName}`;
    activeUsers.push({ ...user, fullName });
  }
}
activeUsers.sort((a, b) => a.fullName.localeCompare(b.fullName));

const div0 = document.createElement('div');
div0.innerHTML = '<h3>Style procédural</h3>';
activeUsers.forEach(user => {
  const p = document.createElement('p');
  p.textContent = user.fullName;
  div0.appendChild(p);
});
document.body.appendChild(div0);
```

⬇️⬇️⬇️

```ts
// Style de programmation déclaratif
import { from } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const out3 = document.createElement('div');
out3.innerHTML = '<h3>Composabilité</h3>';
document.body.appendChild(out3);

const users3 = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const filterActive = filter((user: any) => user.status === 'active');
const formatFullName = map((user: any) => ({ ...user, fullName: `${user.firstName} ${user.lastName}` }));
const collectAndSort = toArray();
const sortByName = map((users: any[]) => users.sort((a, b) => a.fullName.localeCompare(b.fullName)));

from(users3).pipe(
  filterActive,
  formatFullName,
  collectAndSort,
  sortByName
).subscribe(users => {
  users.forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.fullName;
    out3.appendChild(div);
  });
});
```

## Techniques d'optimisation du pipeline

### 1. Importance de l'ordre des opérateurs

L'ordre des opérateurs a un impact significatif sur les performances et les fonctionnalités.

```ts
// Inefficace : map est appliqué à tous les éléments
observable$.pipe(
  map(x => expensiveTransformation(x)),
  filter(x => x > 10)
)

// Efficace : filter est exécuté en premier, réduisant le nombre d'éléments à transformer
observable$.pipe(
  filter(x => x > 10),
  map(x => expensiveTransformation(x))
)
```

### 2. Création de pipelines personnalisés

Les processus complexes peuvent être extraits en pipelines réutilisables.

```ts
import { Observable, pipe } from 'rxjs';
import { filter, map } from 'rxjs';

// Fonction de pipeline personnalisé
export function filterAndTransform<T, R>(
  filterFn: (value: T) => boolean,
  transformFn: (value: T) => R
) {
  return pipe(
    filter(filterFn),
    map(transformFn)
  );
}

// Exemple d'utilisation
observable$.pipe(
  filterAndTransform(
    x => x > 10,
    x => x * 2
  )
).subscribe(console.log);
```

## Erreurs courantes avec les pipelines

### 1. Erreur d'ordre des opérateurs

```ts
// ❌ Si filter est appliqué avant debounceTime,
// filter est exécuté sur chaque entrée, réduisant l'effet du debounce
inputEvents$.pipe(
  filter(text => text.length > 2),
  debounceTime(300)
)

// ✅ Appliquer d'abord debounceTime
inputEvents$.pipe(
  debounceTime(300),
  filter(text => text.length > 2)
)
```

### 2. Effets secondaires dans le pipeline

```ts
// ❌ Exécuter les effets de bord directement dans le pipeline
observable$.pipe(
  map(data => {
    // Effet de bord (mauvais exemple)
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
    return data;
  })
)

// ✅ Utiliser l'opérateur tap
observable$.pipe(
  tap(data => {
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
  }),
  // La conversion des données se fait avec map
  map(data => transformData(data))
)
```

## Résumé

Les pipelines RxJS sont un mécanisme puissant pour gérer des flux de données asynchrones complexes d'une manière déclarative et composable. Des pipelines bien conçus peuvent améliorer de manière significative la lisibilité, la maintenabilité et la réutilisation du code.

Lors de la conception des pipelines, il convient de garder à l'esprit les points suivants :

1. Choisir la séquence d'opérateurs la plus efficace
2. Extraire et réutiliser les modèles de pipeline courants
3. Isoler les effets secondaires à l'aide de l'opérateur `tap`
4. S'assurer que chaque étape du pipeline a une responsabilité unique

Une telle approche orientée pipeline est particulièrement puissante dans des scénarios tels que le traitement d'événements complexes de l'interface utilisateur, les requêtes API et la gestion d'état.
