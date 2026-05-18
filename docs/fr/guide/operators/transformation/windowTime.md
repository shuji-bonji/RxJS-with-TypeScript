---
description: "L'opérateur windowTime peut fractionner un Observable à intervalles de temps réguliers et traiter les valeurs émises dans chaque intervalle de temps comme des Observables distincts. Cette section décrit le fractionnement temporel des flux, le traitement par lots, les différences par rapport à bufferTime et la mise en œuvre à sécurité de type dans TypeScript."
---

# windowTime - diviser l'Observable à intervalles réguliers

L'opérateur `windowTime` regroupe les valeurs de l'Observable source **à intervalles réguliers** et produit le groupe **en tant que nouvel Observable**.
Alors que `bufferTime` renvoie un tableau, `windowTime` renvoie **Observable\<T>**, permettant à d'autres opérateurs d'être appliqués à chaque fenêtre.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// 100msPublier la valeur chaque
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // 1Fenêtre créée toutes les secondes
  take(3),          // Première3Une seule fenêtre
  mergeAll()        // Aplatir chaque fenêtre
).subscribe(value => {
  console.log('Valeur:', value);
});

// Sortie:
// 1Secondes: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2Secondes: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3Secondes: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

- Une nouvelle fenêtre (Observable) est créée tous les temps spécifiés (1000 ms).
- Chaque fenêtre peut être traitée comme une Observable indépendante.

[🌐 Official RxJS documentation - `windowTime`](https://rxjs.dev/api/operators/windowTime)

## 💡 Modèle d'utilisation typique.

- **Traitement par lots basé sur le temps** : les données sont traitées par lots à intervalles de temps réguliers.
- Agrégation de données en temps réel** : comptage du nombre d'événements par seconde
- Contrôle des performances** : collecte de paramètres à intervalles réguliers.
- Analyse des données de séries temporelles : traitement statistique par période de temps.

## 🔍 Différences avec bufferTime

TABLEAU_11___.

```ts
import { interval } from 'rxjs';
import { bufferTime, windowTime, take } from 'rxjs';

const source$ = interval(100);

// bufferTime - Sortie sous forme de tableau
source$.pipe(
  bufferTime(1000),
  take(2)
).subscribe(values => {
  console.log('Tampon (tableau):', values);
  // Sortie: Tampon (tableau): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// windowTime - Observable Sortie en tant que
source$.pipe(
  windowTime(1000),
  take(2)
).subscribe(window$ => {
  console.log('Fenêtre (Observable):', window$);
  // Emboîté subscribe: window Modèles requis par la spécification de l'opérateur du système
  // (Voir aussiwindowTime A l'intérieur émis par Observable requis pour consommer)
  window$.subscribe(value => {
    console.log('  Valeur:', value);
  });
});
```

## 🧠 Exemple de code pratique 1 : Comptage des clics par seconde

Cet exemple montre comment compter le nombre de clics sur un bouton toutes les secondes.

```ts
import { fromEvent } from 'rxjs';
import { windowTime, map, mergeAll, scan } from 'rxjs';

// Création de boutons
const button = document.createElement('button');
button.textContent = 'Cliquer';
document.body.appendChild(button);

// Zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Événement de clic
const clicks$ = fromEvent(button, 'click');

let windowNumber = 0;

clicks$.pipe(
  windowTime(1000), // 1Fenêtre créée toutes les secondes
  map(window$ => {
    ++windowNumber;

    // Compter le nombre de clics dans chaque fenêtre
    return window$.pipe(
      scan(count => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  const timestamp = new Date().toLocaleTimeString();
  output.textContent = `[${timestamp}] Fenêtre ${windowNumber}: ${count}Cliqué une fois`;
});
```

- Une nouvelle fenêtre est créée chaque seconde.
- Le nombre de clics dans chaque fenêtre est compté en temps réel.

## 🎯 Exemple de code pratique 2 : Statistiques par période de temps

Cet exemple calcule la somme et la moyenne des valeurs pour chaque période de temps.

```ts
import { interval } from 'rxjs';
import { windowTime, map, mergeMap, toArray, take } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Traitement statistique pour chaque période de temps (toutes les1(chaque seconde)</h3>';
document.body.appendChild(output);

const table = document.createElement('table');
table.style.borderCollapse = 'collapse';
table.style.marginTop = '10px';
table.innerHTML = `
  <thead>
    <tr style="background: #f0f0f0;">
      <th style="border: 1px solid #ccc; padding: 8px;">Fenêtre</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Nombre de cas</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Total</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Moyenne</th>
    </tr>
  </thead>
  <tbody id="stats-body"></tbody>
`;
output.appendChild(table);

const source$ = interval(100).pipe(
  map(() => Math.floor(Math.random() * 100)) // Valeur aléatoire
);

let windowNumber = 0;

source$.pipe(
  windowTime(1000), // 1(chaque seconde)
  take(5),          // 5Une seule fenêtre
  mergeMap(window$ => {
    const current = ++windowNumber;

    // Les valeurs de chaque fenêtre sont converties en tableau pour le traitement statistique.
    return window$.pipe(
      toArray(),
      map(values => ({
        window: current,
        count: values.length,
        sum: values.reduce((a, b) => a + b, 0),
        avg: values.length > 0
          ? (values.reduce((a, b) => a + b, 0) / values.length).toFixed(2)
          : 0
      }))
    );
  })
).subscribe(stats => {
  const tbody = document.getElementById('stats-body')!;
  const row = document.createElement('tr');
  row.innerHTML = `
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.window}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.count}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.sum}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.avg}</td>
  `;
  tbody.appendChild(row);
});
```

- Les statistiques de chaque fenêtre peuvent être calculées individuellement.
- Différents processus peuvent être appliqués à chaque fenêtre.
- Présentation visuelle des données statistiques sous forme de tableau.

## 📊 Fenêtres qui se chevauchent (windowCreationInterval)

Vous pouvez faire se chevaucher des fenêtres en spécifiant `windowCreationInterval` comme second argument.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray, take, map } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Fenêtres qui se chevauchent</h3>';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.marginTop = '10px';
document.body.appendChild(output);

const source$ = interval(100);

source$.pipe(
  windowTime(
    2000,  // Longueur de la fenêtre: 2secondes
    1000   // Intervalle entre les créations de fenêtres: 1secondes
  ),
  take(3),
  mergeMap((window$, index) =>
    window$.pipe(
      toArray(),
      map(values => ({ window: index + 1, values }))
    )
  )
).subscribe(result => {
  const div = document.createElement('div');
  div.style.marginTop = '10px';
  div.style.padding = '5px';
  div.style.backgroundColor = '#f5f5f5';
  div.style.borderLeft = '3px solid #4CAF50';

  const title = document.createElement('strong');
  title.textContent = `Fenêtre ${result.window}:`;
  div.appendChild(title);

  div.appendChild(document.createElement('br'));

  const values = document.createElement('span');
  values.textContent = `Valeur: [${result.values.join(', ')}]`;
  div.appendChild(values);

  div.appendChild(document.createElement('br'));

  const info = document.createElement('span');
  info.style.color = '#666';
  info.textContent = `(${result.values.length}Valeurs individuelles,${(result.window - 1)}secondes à${(result.window + 1)}secondes)`;
  div.appendChild(info);

  output.appendChild(div);

  // ChromeMesure: Rendu forcé
  void output.offsetHeight;
});
```

**Description du comportement:**
- Fenêtre 1** : valeurs de 0 à 2 secondes `[0, 1, 2, ... , 19]` (20 valeurs)
- Fenêtre 2** : valeurs de 1 à 3 secondes `[10, 11, 12, ... , 29]` (20) ← Les valeurs 10 à 19 se chevauchent avec la fenêtre 1.
- Fenêtre 3** : valeurs de 2 à 4 secondes `[20, 21, 22, ... , 39]` (20 valeurs) ← Les valeurs 20-29 se chevauchent avec la fenêtre 2.

- La création d'une nouvelle fenêtre avec un intervalle (1 seconde) plus court que la longueur de la fenêtre (2 secondes) entraînera une duplication.
- Ceci est utile pour mettre en œuvre des fenêtres coulissantes.

## 🎯 Exemple pratique : surveillance d'événements en temps réel

```ts
import { fromEvent } from 'rxjs';
import { windowTime, mergeMap, toArray, map } from 'rxjs';

// Zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Surveillance du mouvement de la souris (5(chaque seconde)</h3>';
document.body.appendChild(output);

const list = document.createElement('ul');
output.appendChild(list);

// Événement de déplacement de la souris
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  windowTime(5000), // 5(chaque seconde)
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(events => ({
        count: events.length,
        timestamp: new Date().toLocaleTimeString()
      }))
    )
  )
).subscribe(result => {
  const item = document.createElement('li');
  item.textContent = `[${result.timestamp}] Mouvement de la souris: ${result.count}temps`;
  list.insertBefore(item, list.firstChild);

  // Maximum10Affichage d'un maximum de
  while (list.children.length > 10) {
    list.removeChild(list.lastChild!);
  }
});
```

## ⚠️ Notes.

### 1. gestion des abonnements aux fenêtres

Chaque fenêtre est un Observable indépendant et doit faire l'objet d'un abonnement explicite.

```ts
source$.pipe(
  windowTime(1000)
).subscribe(window$ => {
  // Emboîté subscribe: window Modèles requis par la spécification de l'opérateur du système
  // Aucune valeur ne circule à moins que la fenêtre elle-même ne soit abonnée à l'événement de mouvement de la souris.
  window$.subscribe(value => {
    console.log('Valeur:', value);
  });
});
```

ou aplatie en utilisant `mergeAll()`, `concatAll()`, `switchAll()`, etc.

```ts
source$.pipe(
  windowTime(1000),
  mergeAll() // Fusionner toutes les fenêtres
).subscribe(value => {
  console.log('Valeur:', value);
});
```

### 2. gestion de la mémoire

Il est important de se désabonner correctement lors d'un fonctionnement prolongé.

```ts
import { takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

const destroy$ = new Subject<void>();

source$.pipe(
  windowTime(1000),
  mergeAll(),
  takeUntil(destroy$) // Désabonnement en cas de destruction
).subscribe();

// par exemple, lorsqu'un composant est détruit
destroy$.next();
destroy$.complete();
```

### Spécifier la valeur maximale (maxWindowSize)

Le troisième argument permet de limiter le nombre maximum de valeurs pour chaque fenêtre.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// 100msPublier la valeur chaque
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // 1Fenêtre créée toutes les secondes
  take(3),          // Première3Une seule fenêtre
  mergeAll()        // Aplatir chaque fenêtre
).subscribe(value => {
  console.log('Valeur:', value);
});

// Sortie:
// 1Secondes: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2Secondes: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3Secondes: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

## 🆚 Comparaison des opérateurs basés sur les fenêtres


```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// 100msPublier la valeur chaque
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // 1Fenêtre créée toutes les secondes
  take(3),          // Première3Une seule fenêtre
  mergeAll()        // Aplatir chaque fenêtre
).subscribe(value => {
  console.log('Valeur:', value);
});

// Sortie:
// 1Secondes: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2Secondes: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3Secondes: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

## 📚 Opérateurs apparentés.

- **[bufferTime](. /bufferTime)** - regroupe les valeurs sous forme de tableau (version tableau de windowTime).
- **[window](. /window)** - divise la fenêtre en émettant un Observable.
- **[windowCount](. /windowCount)** - partitionnement de la fenêtre sur la base d'un nombre de pièces.
- **[windowToggle](. /windowToggle)** - contrôle de la fenêtre avec Observable de début et de fin.
- **[windowWhen](. /windowWhen)** - découpage de fenêtres avec conditions de fermeture dynamiques

## Résumé.

L'opérateur `windowTime` est un outil puissant pour diviser les flux sur une base temporelle et traiter chaque période comme une Observable indépendante.

- ✅ Crée automatiquement des fenêtres à intervalles réguliers
- ✅ Différents traitements peuvent être appliqués à chaque fenêtre
- ✅ Les fenêtres coulissantes (duplicatas) sont prises en charge
- Idéal pour l'agrégation et l'analyse de données en temps réel
- ⚠️ Gestion de l'abonnement requise
- ⚠️ Gestion de la mémoire
