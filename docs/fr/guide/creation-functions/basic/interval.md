---
description: "interval() - Fonction de création qui émet continuellement des valeurs à des intervalles spécifiés : Essentielle pour le polling, les tâches périodiques, les comptes à rebours et les mises à jour en temps réel"
---

# interval() - Émission continue à intervalles spécifiés

`interval()` est une fonction de création qui émet en continu des valeurs à des intervalles de temps spécifiés.

## Vue d'ensemble

`interval()` émet en continu des nombres consécutifs à partir de 0 à des intervalles spécifiés en millisecondes. Elle est fréquemment utilisée pour les processus de polling et l'exécution de tâches périodiques.

**Signature** :
```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

**Documentation officielle** : [📘 RxJS Official : interval()](https://rxjs.dev/api/index/function/interval)

## Utilisation de base

`interval()` émet des nombres qui s'incrémentent à un intervalle spécifié.

```typescript
import { interval } from 'rxjs';

// Émettre des valeurs toutes les 1 seconde
const interval$ = interval(1000);

interval$.subscribe(value => {
  console.log('Valeur:', value);
});

// Sortie (toutes les 1 seconde):
// Valeur: 0
// Valeur: 1
// Valeur: 2
// Valeur: 3
// ... (continue indéfiniment)
```

## Caractéristiques importantes

### 1. Nombres consécutifs à partir de 0

`interval()` émet toujours des nombres entiers qui commencent à 0 et s'incrémentent de 1.

```typescript
import { interval } from 'rxjs';
import { take } from 'rxjs';

interval(500).pipe(
  take(5) // Obtenir seulement les 5 premières valeurs
).subscribe(value => console.log(value));

// Sortie (toutes les 500ms):
// 0
// 1
// 2
// 3
// 4
```

### 2. Ne se termine jamais (flux infini)

`interval()` ne se termine pas automatiquement et **doit être désabonné**.

```typescript
import { interval } from 'rxjs';

const subscription = interval(1000).subscribe(value => {
  console.log('Valeur:', value);
});

// Désabonnement après 5 secondes
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Arrêté');
}, 5000);
```

> [!WARNING]
> **Oublier de se désabonner provoque des fuites de mémoire**
>
> Comme `interval()` continue d'émettre des valeurs indéfiniment, oublier de se désabonner peut provoquer des fuites de mémoire et des problèmes de performance. Assurez-vous d'appeler `unsubscribe()` ou d'utiliser des opérateurs tels que `take()`, `takeUntil()`, ou `takeWhile()` pour terminer automatiquement.

### 3. Cold Observable

`interval()` est un Cold Observable, qui crée un timer indépendant pour chaque abonnement.

```typescript
import { interval } from 'rxjs';

const interval$ = interval(1000);

// Abonnement 1
interval$.subscribe(value => console.log('Observateur 1:', value));

// Ajout de l'abonnement 2 après 2 secondes
setTimeout(() => {
  interval$.subscribe(value => console.log('Observateur 2:', value));
}, 2000);

// Sortie:
// Observateur 1: 0
// Observateur 1: 1
// Observateur 2: 0  ← Commence à 0 avec un timer indépendant
// Observateur 1: 2
// Observateur 2: 1
```

> [!NOTE]
> **Caractéristiques du Cold Observable** :
> - Une exécution indépendante est lancée pour chaque abonnement
> - Chaque abonné reçoit son propre flux de données
> - Un timer indépendant est démarré pour chaque abonnement ; utilisez `share()` si vous avez besoin de partager les données
>
> Voir [Cold Observable et Hot Observable](/fr/guide/observables/cold-and-hot-observables) pour plus d'informations.

## Différence entre interval() et timer()

Bien que `interval()` et `timer()` soient similaires, il y a quelques différences importantes.

```typescript
import { interval, timer } from 'rxjs';
import { take } from 'rxjs';

// interval() - démarre immédiatement, émission continue
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - démarre après un délai
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// Sortie:
// interval: 0  (après 1 seconde)
// interval: 1  (après 2 secondes)
// timer: 0     (après 2 secondes)
// interval: 2  (après 3 secondes)
// timer: 1     (après 3 secondes)
// timer: 2     (après 4 secondes)
```

| Fonction de création | Moment de démarrage | Objectif |
|-------------------|--------------|---------|
| `interval(1000)` | Démarre immédiatement (première valeur après 1 seconde) | Exécution périodique |
| `timer(2000, 1000)` | Démarre après le temps spécifié | Exécution périodique avec délai |
| `timer(2000)` | Émet une seule fois après le temps spécifié | Exécution différée |

## Cas d'utilisation pratiques

### 1. Polling d'API

Appeler l'API à intervalles réguliers pour mettre à jour les données.

```typescript
import { from, interval } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

function fetchStatus(): Promise<Status> {
  return fetch('https://jsonplaceholder.typicode.com/users/1')
    .then(res => res.json());
}

// Interroger l'API toutes les 5 secondes
const polling$ = interval(5000).pipe(
  switchMap(() => from(fetchStatus())),
  catchError(error => {
    console.error('Erreur API:', error);
    return of({ status: 'error', timestamp: Date.now() });
  })
);

const subscription = polling$.subscribe(data => {
  console.log('Mise à jour du statut:', data);
});

// Arrêter si nécessaire
// subscription.unsubscribe();
```

### 2. Compte à rebours

Implémenter un compte à rebours pour une limite de temps.

```typescript
import { interval } from 'rxjs';
import { map, takeWhile } from 'rxjs';

const countdown$ = interval(1000).pipe(
  map(count => 10 - count), // Compte à rebours depuis 10 secondes
  takeWhile(time => time >= 0) // Auto-complétion à 0
);

countdown$.subscribe({
  next: time => console.log(`Temps restant: ${time} secondes`),
  complete: () => console.log('Temps écoulé!')
});

// Sortie (toutes les 1 seconde):
// Temps restant: 10 secondes
// Temps restant: 9 secondes
// ...
// Temps restant: 0 secondes
// Temps écoulé!
```

### 3. Fonction de sauvegarde automatique

Sauvegarder automatiquement le contenu du formulaire périodiquement.

```typescript
import { fromEvent, from } from 'rxjs';
import { switchMap, debounceTime } from 'rxjs';

// Créer un formulaire
const form = document.createElement('form');
form.id = 'myForm';
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Entrez du texte';
form.appendChild(input);
document.body.appendChild(form);

const input$ = fromEvent(form, 'input');

// Sauvegarde automatique 3 secondes après l'arrêt de la saisie (raccourci pour la démo)
input$.pipe(
  debounceTime(3000), // S'il n'y a pas de saisie pendant 3 secondes
  switchMap(() => {
    const formData = new FormData(form);
    // Démo: Simuler avec Promise au lieu d'une vraie API
    return from(
      Promise.resolve({ success: true, data: formData.get('text') })
    );
  })
).subscribe(result => {
  console.log('Sauvegarde automatique:', result);
});
```

### 4. Affichage d'horloge en temps réel

Mettre à jour l'heure actuelle en temps réel.

```typescript
import { interval } from 'rxjs';
import { map } from 'rxjs';

// Créer un élément pour l'affichage de l'horloge
const clockElement = document.createElement('div');
clockElement.id = 'clock';
clockElement.style.fontSize = '24px';
clockElement.style.fontFamily = 'monospace';
clockElement.style.padding = '20px';
document.body.appendChild(clockElement);

const clock$ = interval(1000).pipe(
  map(() => new Date().toLocaleTimeString())
);

clock$.subscribe(time => {
  clockElement.textContent = time;
});

// Sortie: L'heure actuelle se met à jour chaque seconde
```

## Utilisation dans un pipeline

`interval()` est utilisé comme point de départ pour les pipelines ou comme déclencheur de contrôle temporel.

```typescript
import { interval } from 'rxjs';
import { map, filter, scan } from 'rxjs';

// Compter uniquement les secondes paires
interval(1000).pipe(
  filter(count => count % 2 === 0),
  scan((sum, count) => sum + count, 0),
  map(sum => `Somme des pairs: ${sum}`)
).subscribe(console.log);

// Sortie (toutes les 1 seconde):
// Somme des pairs: 0
// Somme des pairs: 2  (0 + 2)
// Somme des pairs: 6  (0 + 2 + 4)
// Somme des pairs: 12 (0 + 2 + 4 + 6)
```

## Erreurs courantes

### 1. Oublier de se désabonner

```typescript
// ❌ Incorrect - s'exécute indéfiniment sans désabonnement
import { interval } from 'rxjs';

function startPolling() {
  interval(1000).subscribe(value => {
    console.log('Valeur:', value); // S'exécute pour toujours
  });
}

startPolling();

// ✅ Correct - conserver l'abonnement et se désabonner si nécessaire
import { interval, Subscription } from 'rxjs';

let subscription: Subscription | null = null;

function startPolling() {
  subscription = interval(1000).subscribe(value => {
    console.log('Valeur:', value);
  });
}

function stopPolling() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startPolling();
// Appeler stopPolling() si nécessaire
```

### 2. Les abonnements multiples créent des timers indépendants

```typescript
// ❌ Non intentionnel - deux timers indépendants sont créés
import { interval } from 'rxjs';

const interval$ = interval(1000);

interval$.subscribe(value => console.log('Observateur 1:', value));
interval$.subscribe(value => console.log('Observateur 2:', value));
// Deux timers s'exécutent en parallèle

// ✅ Correct - partager un seul timer
import { interval } from 'rxjs';
import { share } from 'rxjs';

const interval$ = interval(1000).pipe(share());

interval$.subscribe(value => console.log('Observateur 1:', value));
interval$.subscribe(value => console.log('Observateur 2:', value));
// Un seul timer est partagé
```

## Considérations sur les performances

Bien que `interval()` soit léger, les performances doivent être prises en compte lors d'exécutions à intervalles courts.

> [!TIP]
> **Conseils d'optimisation** :
> - Ne pas effectuer de traitement inutile (affiner avec `filter()`)
> - Utiliser les intervalles courts (moins de 100ms) avec prudence
> - S'assurer que les abonnements sont désabonnés
> - Si plusieurs Observateurs sont nécessaires, les partager avec `share()`

```typescript
import { interval } from 'rxjs';
import { filter, share } from 'rxjs';

// ❌ Problème de performance - traitement lourd toutes les 100ms
interval(100).subscribe(() => {
  // Traitement lourd
  heavyCalculation();
});

// ✅ Optimisation - traiter uniquement lorsque nécessaire
interval(100).pipe(
  filter(count => count % 10 === 0), // Une fois par seconde (une fois toutes les 10 fois)
  share() // Partager entre plusieurs Observateurs
).subscribe(() => {
  heavyCalculation();
});
```

## Fonctions de création associées

| Fonction | Différence | Utilisation |
|----------|------|----------|
| **[timer()](/fr/guide/creation-functions/basic/timer)** | Démarre après un délai, ou n'émet qu'une seule fois | Exécution différée ou traitement unique |
| **[fromEvent()](/fr/guide/creation-functions/basic/fromEvent)** | Piloté par événement | Traitement selon les opérations utilisateur |
| **range()** | Émet immédiatement les nombres dans la plage spécifiée | Lorsque le contrôle temporel n'est pas nécessaire |

## Résumé

- `interval()` émet continuellement des valeurs à des intervalles spécifiés
- Émet des entiers consécutifs à partir de 0
- Ne s'auto-complète pas, doit être désabonné
- Fonctionne comme un Cold Observable (timer indépendant pour chaque abonnement)
- Idéal pour le polling, l'exécution périodique, le compte à rebours, etc.

## Prochaines étapes

- [timer() - Commencer à émettre après un délai](/fr/guide/creation-functions/basic/timer)
- [fromEvent() - Convertir les événements en Observable](/fr/guide/creation-functions/basic/fromEvent)
- [Retour aux fonctions de création de base](/fr/guide/creation-functions/basic/)
