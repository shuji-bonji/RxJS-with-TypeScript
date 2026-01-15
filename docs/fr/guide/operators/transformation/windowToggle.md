---
description: "windowToggle est un opérateur de transformation RxJS avancé qui contrôle les déclencheurs de début et de fin avec des Observables séparés, permettant de gérer indépendamment plusieurs périodes de fenêtre. Idéal pour la collecte de données pendant les heures de bureau ou l'enregistrement d'événements pendant l'appui sur un bouton, où une gestion de période dynamique est nécessaire. L'inférence de type TypeScript permet un traitement de division de fenêtre type-safe."
---

# windowToggle - Fenêtre avec déclencheurs

L'opérateur `windowToggle` contrôle les **déclencheurs de début** et **de fin** avec des Observables séparés et émet chaque période comme un nouvel Observable. C'est un opérateur de fenêtre avancé capable de gérer plusieurs périodes de fenêtre simultanément.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // Émet toutes les 0.5 secondes

// Déclencheur de début : toutes les 2 secondes
const opening$ = interval(2000);

// Déclencheur de fin : 1 seconde après le début
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre :', value);
});

// Début à 2s, fin à 3s → valeurs : 4, 5
// Début à 4s, fin à 5s → valeurs : 8, 9
// Début à 6s, fin à 7s → valeurs : 12, 13
```

**Flux d'opération** :
1. `opening$` émet une valeur → début de la fenêtre
2. L'Observable retourné par `closing()` émet une valeur → fin de la fenêtre
3. Plusieurs périodes de fenêtre peuvent se chevaucher

[🌐 Documentation officielle RxJS - `windowToggle`](https://rxjs.dev/api/operators/windowToggle)

## 💡 Patterns d'utilisation typiques

- Collecte de données pendant les heures de bureau
- Enregistrement d'événements pendant l'appui sur un bouton
- Suivi des actions pendant les sessions actives
- Traitement de flux nécessitant une gestion de période dynamique

## 🔍 Différences avec bufferToggle

| Opérateur | Sortie | Cas d'utilisation |
|:---|:---|:---|
| `bufferToggle` | **Tableau (T[])** | Traitement groupé des valeurs |
| `windowToggle` | **Observable\<T>** | Traitement de flux différent par groupe |

## 🧠 Exemple de code pratique : enregistrement pendant l'appui sur un bouton

Exemple d'enregistrement de données entre mousedown et mouseup.

```ts
import { fromEvent, interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Maintenir';
document.body.appendChild(button);

// Zone de sortie
const display = document.createElement('div');
display.style.marginTop = '10px';
document.body.appendChild(display);

// Flux de données (toutes les 100ms)
const data$ = interval(100);

// Début : mousedown
const mouseDown$ = fromEvent(button, 'mousedown');

// Fin : mouseup
const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

data$.pipe(
  windowToggle(mouseDown$, mouseUp),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(events => {
  display.textContent = `Événements enregistrés pendant le maintien : ${events.length}`;
  console.log('Données enregistrées :', events);
});
```

## 🎯 Périodes de fenêtre qui se chevauchent

Une caractéristique de `windowToggle` est qu'il peut gérer plusieurs périodes de fenêtre simultanément.

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Début : toutes les secondes
const opening$ = interval(1000);

// Fin : 1.5 secondes après le début
const closing = () => interval(1500);

source$.pipe(
  windowToggle(opening$, closing),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenêtre :', values);
});

// Sortie :
// Fenêtre : [4, 5, 6, 7]       (début 1s → fin 2.5s)
// Fenêtre : [9, 10, 11, 12]    (début 2s → fin 3.5s)
// Fenêtre : [14, 15, 16, 17]   (début 3s → fin 4.5s)
```

## ⚠️ Points d'attention

### 1. Attention aux fuites de mémoire

Si le déclencheur de début est trop fréquent, de nombreuses fenêtres existeront simultanément, consommant de la mémoire.

```ts
// ❌ Mauvais exemple : début toutes les 100ms, fin 5s plus tard
const opening$ = interval(100); // Trop fréquent
const closing = () => interval(5000);
// Jusqu'à 50 fenêtres peuvent exister simultanément → risque mémoire

// ✅ Bon exemple : intervalles appropriés
const opening$ = interval(2000); // Toutes les 2 secondes
const closing = () => interval(1000); // Fenêtre pendant 1 seconde
// Maximum 1-2 fenêtres simultanées
```

### 2. Chevauchement des périodes de fenêtre

Les périodes de fenêtre qui se chevauchent font que les mêmes valeurs sont incluses dans plusieurs fenêtres. Vérifiez si c'est le comportement souhaité.

## 📚 Opérateurs associés

- [`bufferToggle`](./bufferToggle) - Regroupe les valeurs en tableau (version tableau de windowToggle)
- [`window`](./window) - Division de fenêtre par timing d'un autre Observable
- [`windowTime`](./windowTime) - Division de fenêtre basée sur le temps
- [`windowCount`](./windowCount) - Division de fenêtre basée sur le nombre
- [`windowWhen`](./windowWhen) - Division de fenêtre avec condition de fermeture dynamique

## Résumé

L'opérateur `windowToggle` est un outil avancé qui contrôle le début et la fin indépendamment et peut traiter chaque période comme un Observable séparé.

- ✅ Peut contrôler le début et la fin séparément
- ✅ Peut gérer plusieurs fenêtres simultanément
- ✅ Peut appliquer un traitement différent à chaque fenêtre
- ⚠️ Nécessite une gestion des abonnements
- ⚠️ Les déclencheurs de début fréquents consomment de la mémoire
- ⚠️ Attention au chevauchement des périodes de fenêtre
