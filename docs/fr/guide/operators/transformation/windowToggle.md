---
description: "windowToggle est un opérateur de conversion RxJS avancé qui permet de gérer indépendamment plusieurs périodes de fenêtres, avec des déclencheurs de début et de fin contrôlés par des Observable distincts. Il est idéal pour les situations où une gestion dynamique des périodes est nécessaire, comme la collecte de données pendant les heures d'ouverture ou l'enregistrement d'événements pendant les pressions sur un bouton, et l'inférence de type TypeScript garantit des opérations de fractionnement de fenêtre sûres du point de vue du type."
---

# windowToggle - démarrer/terminer la fenêtre de contrôle

L'opérateur `windowToggle` contrôle le **start trigger** et le **end trigger** avec des Observable séparés, émettant chaque période comme un nouvel Observable. Il s'agit d'un opérateur de fenêtre avancé qui peut gérer plusieurs périodes de fenêtre simultanément.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // 0.5Émission d'une valeur toutes les secondes

// Déclenchement du démarrage: 2Chaque seconde
const opening$ = interval(2000);

// Déclenchement de fin: Depuis le début1Secondes après
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// 2Commence à la seconde,3Fin à la seconde → Valeur: 4, 5
// 4Commence à la seconde,5Fin à la seconde → Valeur: 8, 9
// 6Commence à la seconde,7Fin à la seconde → Valeur: 12, 13
```

**Flux des opérations** : 1.
1. `opening$` émet une valeur → la fenêtre démarre
2. l'Observable renvoyé par `closing()` délivre une valeur → la fenêtre se termine
3. plusieurs périodes de fenêtre peuvent se chevaucher

[🌐 Official RxJS documentation - `windowToggle`](https://rxjs.dev/api/operators/windowToggle)

## 💡 Modèle d'utilisation typique.

- Collecte de données pendant les heures de bureau
- Enregistrement d'événements pendant les pressions sur les boutons
- Suivi des actions pendant les sessions actives
- Traitement des flux nécessitant une gestion dynamique des périodes

## 🔍 Différences avec bufferToggle

TABLEAU 12___.

```ts
import { interval } from 'rxjs';
import { bufferToggle, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500);
const opening$ = interval(2000);
const closing = () => interval(1000);

// bufferToggle - Sortie sous forme de tableau
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(values => {
  console.log('Buffer (tableau):', values);
  // Sortie en tant que: Buffer (tableau): [4, 5]
});

// windowToggle - Observable Sortie en tant que
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  console.log('Fenêtre (Observable):', window$);
  // Nid subscribe: window Modèles requis par la spécification de l'opérateur du système
  // (Voir aussiwindowToggle A l'intérieur émis par Observable requis pour la consommation)
  window$.subscribe(value => {
    console.log('  Valeur dans la fenêtre:', value);
  });
});
```

## 🧠 Exemple de code pratique 1 : Enregistrement d'événements lors de l'appui sur un bouton

Voici un exemple d'enregistrement de données entre le passage de la souris vers le bas et le passage de la souris vers le haut.

```ts
import { fromEvent, interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

// Création de boutons
const button = document.createElement('button');
button.textContent = 'Maintien';
document.body.appendChild(button);

// Zone de sortie
const display = document.createElement('div');
display.style.marginTop = '10px';
document.body.appendChild(display);

// Flux de données (100mspar flux)
const data$ = interval(100);

// Démarrage: Souris vers le bas
const mouseDown$ = fromEvent(button, 'mousedown');

// Fin: Souris vers le haut
const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

data$.pipe(
  windowToggle(mouseDown$, mouseUp),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(events => {
  display.textContent = `Événements enregistrés pendant la mise en attente: ${events.length}Sujet`;
  console.log('Données enregistrées:', events);
});
```

## 🎯 Exemple de code pratique 2 : Collecte de données pendant les heures de bureau

Il s'agit d'un exemple de collecte de données de capteur depuis le début de l'activité jusqu'à la fin des heures de bureau.

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, mergeMap, scan, map } from 'rxjs';

// Données du capteur (toujours acquises)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10, // 20-30Degré
    humidity: 40 + Math.random() * 20     // 40-60%
  }))
);

// Début de l'opération: 2Secondes, puis10Chaque seconde
const businessOpen$ = timer(2000, 10000);

// Fin de la session de vente: Depuis le début5Secondes après
const businessClose = () => timer(5000);

let sessionNumber = 0;

sensorData$.pipe(
  windowToggle(businessOpen$, businessClose),
  mergeMap(window$ => {
    const current = ++sessionNumber;
    console.log(`Session de vente ${current} Démarrage`);

    // Calculer les statistiques pour chaque fenêtre
    return window$.pipe(
      scan((stats, data) => ({
        count: stats.count + 1,
        totalTemp: stats.totalTemp + data.temperature,
        totalHumidity: stats.totalHumidity + data.humidity
      }), { count: 0, totalTemp: 0, totalHumidity: 0 }),
      map(stats => ({
        session: current,
        count: stats.count,
        avgTemp: stats.totalTemp / stats.count,
        avgHumidity: stats.totalHumidity / stats.count
      }))
    );
  })
).subscribe(stats => {
  console.log(`Sessions ${stats.session}: Nombre d'échantillons ${stats.count}Sujet`);
  console.log(`  Température moyenne: ${stats.avgTemp.toFixed(1)}°C`);
  console.log(`  Humidité moyenne: ${stats.avgHumidity.toFixed(1)}%`);
});
```

## 🎯 Exemple pratique : gestion des périodes de téléchargement

Voici un exemple de gestion des périodes de téléchargement de données à l'aide de boutons de démarrage et d'arrêt.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { windowToggle, mergeMap, toArray, map } from 'rxjs';

// UICréation d'éléments
const startButton = document.createElement('button');
startButton.textContent = 'Démarrage';
document.body.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Arrêtés';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
document.body.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'En attente...';
document.body.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
document.body.appendChild(result);

// Flux de données (1(Génère des données de téléchargement toutes les secondes)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    size: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp: new Date()
  }))
);

// Déclenche le début et la fin
const start$ = fromEvent(startButton, 'click');
const stop$ = new Subject<void>();

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next();
  status.textContent = 'Arrêté';
  startButton.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe(() => {
  status.textContent = 'Téléchargement en cours...';
  startButton.disabled = true;
  stopButton.disabled = false;
});

// Gestion des fenêtres
downloadData$.pipe(
  windowToggle(start$, () => stop$),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Téléchargement terminé</strong><br>
    Nombre de cas: ${downloads.length}Sujet<br>
    Taille totale: ${(totalSize / 1024).toFixed(2)} MB<br>
    Taille moyenne: ${avgSize.toFixed(0)} KB
  `;
});
```

## 🎯 Chevauchement des périodes de fenêtre

Une des caractéristiques de `windowToggle` est qu'elle peut gérer plusieurs périodes de fenêtres simultanément.

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Démarrage: 1Chaque seconde
const opening$ = interval(1000);

// Fin: Depuis le début1.5Secondes après
const closing = () => interval(1500);

source$.pipe(
  windowToggle(opening$, closing),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenêtre:', values);
});

// Sortie en tant que:
// Fenêtre: [4, 5, 6, 7]       (Voir aussi1(Début de seconde) → 2.5(Fin de la seconde)
// Fenêtre: [9, 10, 11, 12]    (Voir aussi2(Début de seconde) → 3.5(Fin de la seconde)
// Fenêtre: [14, 15, 16, 17]   (Voir aussi3(Début de seconde) → 4.5(Fin de la seconde)
```

**Timeline** :.

```
Source:    0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Démarrage:      ----1Secondes----2Secondes----3Secondes----4Secondes
Période.1:     [------1.5Secondes-----]
            └→ Fenêtre1: [4,5,6,7]
Période.2:            [------1.5Secondes-----]
                   └→ Fenêtre2: [9,10,11,12]
Période.3:                   [------1.5Secondes-----]
                          └→ Fenêtre3: [14,15,16,17]
```

## ⚠️ Notes.

### 1. gestion des abonnements aux fenêtres

Chaque fenêtre est un Observable indépendant et doit être explicitement souscrite ou aplatie avec `mergeAll()` etc.

```ts
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  // Nid subscribe: window Modèles requis par la spécification de l'opérateur du système
  // La valeur ne circulera pas à moins que la fenêtre elle-même ne soit souscrite.
  window$.subscribe(value => {
    console.log('Valeur:', value);
  });
});
```

### Attention aux fuites de mémoire.

Si les déclenchements de démarrage sont trop fréquents, de nombreuses fenêtres existeront en même temps, ce qui consommera de la mémoire.

```ts
// ❌ Mauvais exemple: Si le début est100mschaque, fin chaque5Secondes après
const opening$ = interval(100); // trop souvent
const closing = () => interval(5000);

source$.pipe(
  windowToggle(opening$, closing)
).subscribe();
// En même temps50Existence possible de deux fenêtres en même temps → Risque de mémoire

// ✅ Bon exemple: Fixer des intervalles appropriés
const opening$ = interval(2000); // 2Chaque seconde
const closing = () => interval(1000); // 1secondes
```

### 3. périodes de fenêtres qui se chevauchent

Les périodes de fenêtre qui se chevauchent font que la même valeur est incluse dans plus d'une fenêtre. Vérifiez s'il s'agit du comportement souhaité.

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // 0.5Émission d'une valeur toutes les secondes

// Déclenchement du démarrage: 2Chaque seconde
const opening$ = interval(2000);

// Déclenchement de fin: Depuis le début1Secondes après
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// 2Commence à la seconde,3Fin à la seconde → Valeur: 4, 5
// 4Commence à la seconde,5Fin à la seconde → Valeur: 8, 9
// 6Commence à la seconde,7Fin à la seconde → Valeur: 12, 13
```

```0___

## 🆚 Comparaison des opérateurs basés sur les fenêtres

```

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // 0.5Émission d'une valeur toutes les secondes

// Déclenchement du démarrage: 2Chaque seconde
const opening$ = interval(2000);

// Déclenchement de fin: Depuis le début1Secondes après
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// 2Commence à la seconde,3Fin à la seconde → Valeur: 4, 5
// 4Commence à la seconde,5Fin à la seconde → Valeur: 8, 9
// 6Commence à la seconde,7Fin à la seconde → Valeur: 12, 13
```

```3___

## 🔄 Différences avec windowWhen

```

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, windowWhen, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowToggle: Séparer le début et la fin々Contrôle séparé du début et de la fin
source$.pipe(
  windowToggle(
    interval(1000),          // Déclenchement du démarrage
    () => timer(500)         // Déclenchement de la fin (après le début)500ms(après le début)
  ),
  mergeAll()
).subscribe();

// windowWhen: Contrôle uniquement le moment de la fin (le prochain commence immédiatement après la fin)
source$.pipe(
  windowWhen(() => timer(1000)), // 1Fenêtre toutes les secondes
  mergeAll()
).subscribe();
```

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // 0.5Émission d'une valeur toutes les secondes

// Déclenchement du démarrage: 2Chaque seconde
const opening$ = interval(2000);

// Déclenchement de fin: Depuis le début1Secondes après
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// 2Commence à la seconde,3Fin à la seconde → Valeur: 4, 5
// 4Commence à la seconde,5Fin à la seconde → Valeur: 8, 9
// 6Commence à la seconde,7Fin à la seconde → Valeur: 12, 13
```

```4___

## 📚 Opérateurs apparentés.

- [`bufferToggle`](. /bufferToggle) - résume les valeurs sous forme de tableau (version tableau de windowToggle).
- [`window`](. /window) - fractionnement de la fenêtre à différents moments de l'Observable.
- [`windowTime`](. /windowTime) - fractionnement de la fenêtre en fonction du temps.
- [`windowCount`](. /windowCount) - fractionnement des fenêtres sur la base du nombre de pièces.
- [`windowWhen`](. /windowWhen) - fractionnement des fenêtres sur la base de conditions de fermeture dynamiques.

## Résumé.

L'opérateur `windowToggle` est un outil avancé qui permet de contrôler le début et la fin indépendamment et de traiter chaque période comme un Observable indépendant.

- ✅ Le début et la fin peuvent être contrôlés séparément.
- Plusieurs fenêtres peuvent être gérées simultanément.
- Des traitements différents peuvent être appliqués à chaque fenêtre.
- ⚠️ Gestion de l'abonnement nécessaire
- ⚠️ Les déclenchements fréquents consomment de la mémoire
- ⚠️ Attention au chevauchement des périodes de fenêtres
