---
description: "L'opérateur bufferToggle est un opérateur de mise en mémoire tampon avancé qui permet de contrôler les déclencheurs de début et de fin par des Observable distincts et de gérer indépendamment plusieurs périodes de mise en mémoire tampon."
---

# bufferToggle - début/fin du contrôle de la mémoire tampon

L'opérateur `bufferToggle` contrôle les **start trigger** et **end trigger** avec des Observable séparés et émet les valeurs ensemble dans un tableau. Il s'agit d'un opérateur de mise en mémoire tampon avancé qui peut gérer plusieurs périodes de mise en mémoire tampon simultanément.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // 0.5Émission d'une valeur toutes les secondes

// Déclenchement du démarrage: 2Chaque seconde
const opening$ = interval(2000);

// Déclenchement de fin: Depuis le début1Quelques secondes après
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Sortie:
// [3, 4, 5]     (2Début à la seconde,3(début à la seconde, fin à la seconde)
// [7, 8, 9]     (4Début à la seconde,5(début à la seconde, fin à la seconde)
// [11, 12, 13]  (6Début à la seconde,7(début à la seconde, fin à la seconde)
```

**Flux des opérations** :.
1. `opening$` émet une valeur → la mise en mémoire tampon commence
2. l'Observable retourné par `closing()` émet une valeur → fin de la mise en mémoire tampon, tableau de sortie
3. plusieurs périodes de mise en mémoire tampon peuvent se chevaucher

[🌐 Official RxJS documentation - `bufferToggle`](https://rxjs.dev/api/operators/bufferToggle)

## 🆚 Contraste avec d'autres opérateurs basés sur les tampons

`bufferToggle` est unique par rapport aux autres opérateurs basés sur les tampons en ce qu'il permet **un contrôle indépendant du début et de la fin**.

### Comparaison des différents opérateurs

TABLEAU_10___.

### Comparaison avec des exemples de code

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9chaque seconde.300msÉmis toutes les

// bufferToggle: Contrôle indépendant du début et de la fin
const opening$ = interval(1000); // 1Début toutes les secondes
const closing = () => interval(500); // Depuis le début500msFin après

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Sortie: [3, 4], [6, 7], [9]
//
// Ligne de temps:
// 0ms  300ms 600ms 900ms 1200ms 1500ms 1800ms 2100ms 2400ms 2700ms
// 0    1     2     3     4      5      6      7      8      9
//                  [Début        Fin]  [Début        Fin]  [Début  Fin]
//                  └→ [3,4]           └→ [6,7]           └→ [9]
```

**Utilisation avec d'autres opérateurs** :.
- **`buffer`** → sort un buffer à chaque fois que l'Observable déclencheur émet une valeur.
- **`bufferTime`** → sort automatiquement un tampon à chaque fois que le trigger Observable émet une valeur.
- **`bufferCount`** → produit un tampon lorsque le nombre spécifié de tampons a été accumulé.
- **`bufferToggle`** → contrôle séparé du début et de la fin, possibilité de chevauchement des périodes.

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // 0.5Émission d'une valeur toutes les secondes

// Déclenchement du démarrage: 2Chaque seconde
const opening$ = interval(2000);

// Déclenchement de fin: Depuis le début1Quelques secondes après
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Sortie:
// [3, 4, 5]     (2Début à la seconde,3(début à la seconde, fin à la seconde)
// [7, 8, 9]     (4Début à la seconde,5(début à la seconde, fin à la seconde)
// [11, 12, 13]  (6Début à la seconde,7(début à la seconde, fin à la seconde)
```

> Pour plus d'informations sur chaque opérateur, voir [buffer](. /buffer), [bufferTime](. /bufferTime), [bufferCount](. /bufferCount).

## 💡 Modèles d'utilisation typiques.

1. **Collecte de données pendant les heures de bureau**.


```ts
   import { interval, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Données du capteur (toujours acquises)
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       value: Math.random() * 100
     }))
   );

   // Début de l'opération: 9:00(simulation): 2(après 1,5 seconde)
   const businessOpen$ = timer(2000, 10000); // 2Quelques secondes plus tard, puis10Chaque seconde

   // Fin des heures de fonctionnement: Depuis le début5Quelques secondes après
   const businessClose = () => timer(5000);

   sensorData$.pipe(
     bufferToggle(businessOpen$, businessClose)
   ).subscribe(data => {
     console.log(`Données pendant les heures de fonctionnement: ${data.length}Cas`);
     console.log(`Valeur moyenne: ${(data.reduce((sum, d) => sum + d.value, 0) / data.length).toFixed(2)}`);
   });
   ```

2. **Enregistrement de l'événement pendant l'appui sur le bouton**
   ```ts
   import { fromEvent, interval } from 'rxjs';
   import { bufferToggle, map, take } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Maintien';
   document.body.appendChild(button);

   const display = document.createElement('div');
   display.style.marginTop = '10px';
   document.body.appendChild(display);

   // Flux de données
   const data$ = interval(100).pipe(
     map(i => ({ id: i, timestamp: Date.now() }))
   );

   // Début: Souris vers le bas
   const mouseDown$ = fromEvent(button, 'mousedown');

   // Fin: Souris vers le haut (mousedownSe produit à partir demouseupjusqu'à)
   const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

   data$.pipe(
     bufferToggle(mouseDown$, mouseUp)
   ).subscribe(events => {
     display.textContent = `Événement enregistré pendant la mise en attente: ${events.length}Cas`;
     console.log('Événements enregistrés:', events);
   });
   ```

3. **Action de l'utilisateur actif enregistrée**
   ```ts
   import { fromEvent, merge, timer } from 'rxjs';
    mport { bufferToggle, map } from 'rxjs';

   // Action de l'utilisateur
   const clicks$ = fromEvent(document, 'click').pipe(
     map(() => ({ type: 'click' as const, timestamp: Date.now() }))
   );

   const scrolls$ = fromEvent(window, 'scroll').pipe(
     map(() => ({ type: 'scroll' as const, timestamp: Date.now() }))
   );

   const keypresses$ = fromEvent(document, 'keypress').pipe(
     map(() => ({ type: 'keypress' as const, timestamp: Date.now() }))
   );

   const actions$ = merge(clicks$, scrolls$, keypresses$);

   // Début de l'état actif: Première action
   const activeStart$ = actions$;

   // Fin de l'état actif: 5Pas d'action en secondes
   const activeEnd = () => timer(5000);

   actions$.pipe(
     bufferToggle(activeStart$, activeEnd)
   ).subscribe(bufferedActions => {
     console.log(`Session active: ${bufferedActions.length}Nombre d'actions`);
     const summary = bufferedActions.reduce((acc, action) => {
       acc[action.type] = (acc[action.type] || 0) + 1;
       return acc;
     }, {} as Record<string, number>);
     console.log('Répartition:', summary);
   });
   ```

## 🧠 Exemple de code pratique (gestion des périodes de téléchargement)

Voici un exemple de gestion des périodes de téléchargement de données à l'aide de boutons de démarrage et d'arrêt.

```

ts.
import { interval, fromEvent, Subject } from 'rxjs' ;
import { bufferToggle, map, take } from 'rxjs' ;

// Création d'éléments d'interface utilisateur
const container = document.createElement('div') ;.
document.body.appendChild(container) ;

const title = document.createElement('h3') ;
title.textContent = "Gestion des téléchargements de données" ;
container.appendChild(title) ;

const startButton = document.createElement('button') ;
startButton.textContent = "Démarrer" ;
container.appendChild(startButton) ;

const stopButton = document.createElement('button') ;
stopButton.textContent = 'Stop' ;
stopButton.disabled = true ;
stopButton.style.marginLeft = '10px' ;
container.appendChild(stopButton) ;

const status = document.createElement('div') ;
status.style.marginTop = '10px' ;
status.textContent = "En attente... ;
container.appendChild(status) ;

const result = document.createElement('div') ;
result.style.marginTop = '10px' ;
container.appendChild(result) ;

// Flux de données (téléchargement des données générées toutes les secondes)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id : i,
    size : Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp : new Date())
  }))
) ;

// Déclenchement du début et de la fin
const start$ = fromEvent(startButton, 'click') ;
const stop$ = new Subject() ;

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next() ;
  status.textContent = 'Stopped' ;
  startButton.disabled = false ;
  stopButton.disabled = true ;
}) ;

start$.subscribe(() => {
  status.textContent = 'Téléchargement...' ;
  startButton.disabled = true ;
  stopButton.disabled = false ;
}) ;

// Mise en mémoire tampon
downloadData$.pipe(
  bufferToggle(start$, () => stop$)
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0) ;
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0 ;

  result.innerHTML = `
    <strong>Téléchargements terminés</strong><br>.
    Nombre de téléchargements : ${downloads.length}<br>.
    Taille totale : ${(totalSize / 1024).toFixed(2)} MB<br>.
    Taille moyenne : ${avgSize.toFixed(0)} KB
  ` ;

  console.log('Downloaded data:', downloads) ;
}) ;

```

## 🎯 Chevauchement des périodes de mise en mémoire tampon

`bufferToggle` Il est possible de gérer simultanément plusieurs périodes de mise en mémoire tampon grâce à la fonction de gestion de la mémoire tampon.

```

ts.
import { interval } from 'rxjs' ;
import { bufferToggle, take } from 'rxjs' ;

const source$ = interval(200).pipe(take(20)) ; // 0-19

// démarrage : toutes les 1 secondes
const opening$ = interval(1000) ;

// fin : 1,5 seconde après le début
const closing = () => interval(1500) ;

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log) ;
// Sortie :.
// [4, 5, 6] (début de la 1ère seconde → fin de 2,5 secondes)
// [9, 10, 11, 12] (début de la 2e seconde → fin de 3,5 secondes) * partiellement dupliqué
// [14, 15, 16, 17] (début de la 3e seconde → fin de 4,5 secondes)

```

**Ligne de temps**:
```

Source: 0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Démarrage : ----1 secondes ----2 secondes ----3 secondes ----4 secondes
Période 1 : [------1.5 seconds-----]
            └→ Sortie : [4,5,6].
Période 2 : [------1.5 seconds-----]
                   └→ Sortie : [9,10,11,12].
Période 3 : [------1.5 seconds-----]
                          └→ Sortie : [14,15,16,17]

```

## 📋 Utilisation sûre des types

TypeScript Voici un exemple d'implémentation à sécurité de type qui utilise les génériques dans le code source.

```

ts.
import { Observable, Subject, interval } from 'rxjs' ;
import { bufferToggle, map } from 'rxjs' ;

interface MetricData {
  timestamp : Date ;.
  cpu : nombre ;
  memory : number ; }
}

interface SessionControl {
  start$ : Observable ;
  stop$ : Observable ;
}

class MetricsCollector {
  private startSubject = new Subject(); ;
  private stopSubject = new Subject() ;.

  start() : void {
    this.startSubject.next() ;
  }

  stop() : void {
    this.stopSubject.next() ; }
  }

  collectMetrics(source$ : Observable) : Observable<MetricData[]> {
    return source$.pipe(
      bufferToggle(
        this.startSubject, .
        () => this.stopSubject
      )
    ) ;
  }
}

// Exemple d'utilisation
const metricsStream$ = interval(500).pipe(
  map(() => ({
    timestamp : new Date(), cpu : Math.random() * 100, cpu.
    cpu : Math.random() * 100, memory.
    memory : Math.random() * 100
  } as MetricData))
) ;

const collector = new MetricsCollector() ;.

collector.collectMetrics(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avgCpu = metrics.reduce((sum, m) => sum + m.cpu, 0) / metrics.length ;
    const avgMemory = metrics.reduce((sum, m) => sum + m.memory, 0) / metrics.length ;
    console.log(`période de collecte : ${metrics.length} cases`) ;
    console.log(`Moyenne CPU : ${avgCpu.toFixed(1)}%`) ;
    console.log(`Mémoire moyenne : ${avgMemory.toFixed(1)}%`) ; console.log(`Mémoire moyenne : ${avgMemory.toFixed(1)}%`) ; }
  }
}) ;

// Démarrage après 3 secondes.
setTimeout(() => {
  console.log('Collection started') ;
  collector.start() ;
}, 3000) ;

// Arrêt après 6 secondes
setTimeout(() => {
  console.log('Collection stopped') ;
  collector.stop() ;
}, 6000) ;

```

## 🔄 bufferWhen Différences entre

`bufferToggle` et `bufferWhen` sont similaires, mais diffèrent dans la manière dont elles sont contrôlées.

```

ts.
import { interval, timer } from 'rxjs' ;
import { bufferToggle, bufferWhen } from 'rxjs' ;

const source$ = interval(200) ;.

// bufferToggle : contrôler le début et la fin séparément
source$.pipe(
  bufferToggle(
    interval(1000), // déclenchement de départ
    () => timer(500) // déclenchement de fin (500 ms après le début)
  )
).subscribe(console.log) ;.

// bufferWhen : contrôle uniquement le timing de la fin (le prochain commence immédiatement après la fin)
source$.pipe(
  bufferWhen(() => timer(1000)) // buffer toutes les secondes
).subscribe(console.log) ; .

```

| Opérateur | Contrôle | Période tampon | Cas d'utilisation |
|---|---|---|---|
| `bufferToggle(open$, close)` | Contrôle séparé du début et de la fin | Duplication possible | Début complexe/Conditions de fin |
| `bufferWhen(closing)` | Seule la fin est contrôlée | Continu | Tampon cyclique simple |

## ⚠️ Erreurs courantes

> [!WARNING]
> `bufferToggle` peut gérer plusieurs périodes de tampon simultanément, mais si le déclencheur de démarrage se déclenche fréquemment, de nombreux tampons existeront simultanément et consommeront de la mémoire.

### Erreur.: Les déclenchements de démarrage sont trop fréquents.

```

ts.
import { interval } from 'rxjs' ;
import { bufferToggle } from 'rxjs' ;

const source$ = interval(100) ;.

// ❌ Mauvais exemple : démarrage toutes les 100ms, fin après 5 secondes
const opening$ = interval(100) ; // trop fréquent
const closing$ = () => interval(5000) ;

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log) ;.
// 50 tampons possibles en même temps → risque de perte de mémoire

```

### Correction: Définir des intervalles appropriés

```

ts.
import { interval } from 'rxjs' ;
import { bufferToggle } from 'rxjs' ;

const source$ = interval(100) ;.

// ✅ Bon exemple : commencer au bon intervalle
const opening$ = interval(2000) ; // toutes les 2 secondes
const closing = () => interval(1000) ; // tampon pendant 1 seconde

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log) ; .
// seulement 1-2 tampons au maximum en même temps.
```

## 🎓 Résumé

### Quand doit-on utiliser bufferToggle ?
- ✅ Si vous voulez contrôler le début et la fin indépendamment
- ✅ Si vous souhaitez collecter des données pendant une période de temps limitée, par exemple lors de l'appui sur un bouton.
- Si vous souhaitez gérer plusieurs périodes de mise en mémoire tampon simultanément
- Lorsque vous souhaitez collecter des données dans des conditions complexes, par exemple uniquement pendant les heures de bureau.

### Quand vous devez utiliser buffer/bufferTime/bufferCount.
- ✅ Lorsqu'une simple mise en mémoire tampon périodique est suffisante
- ✅ Lorsqu'un seul déclencheur suffit pour le contrôle

### Quand faut-il utiliser bufferWhen ?
- ✅ Lorsque seule la condition de fin doit être contrôlée dynamiquement.
- ✅ Lorsque des périodes continues de mise en mémoire tampon sont nécessaires.

### Notes.
- ⚠️ Les déclenchements fréquents entraînent l'existence simultanée de nombreux tampons, ce qui consomme de la mémoire.
- ⚠️ Les périodes de mise en mémoire tampon peuvent se chevaucher
- ⚠️ Peut être difficile à déboguer en raison de la complexité des contrôles.

## 🚀 Prochaines étapes.

- **[buffer](. /buffer)** - apprendre les bases de la mise en mémoire tampon.
- **[bufferTime](. /bufferTime)** - apprendre la mise en mémoire tampon basée sur le temps.
- **[bufferCount](. /bufferCount)** - apprendre la mise en mémoire tampon par morceaux **[bufferCount](.
- **[bufferWhen](https://rxjs.dev/api/operators/bufferWhen)** - apprendre le contrôle dynamique de sortie (documentation officielle)
- **[conversion-operator-practical-use-cases](. /practical-use-cases)** - apprendre des cas d'utilisation réels
