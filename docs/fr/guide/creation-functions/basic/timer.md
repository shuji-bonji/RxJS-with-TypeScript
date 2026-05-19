---
description: "timer() - Creation Function qui émet une valeur après un temps spécifié, en utilisant timer(delay) pour une exécution retardée ponctuelle et timer(delay, period) pour une exécution périodique retardée ; comment l'utiliser avec interval() ; inférence de type en TypeScript, Explication de son utilisation comme alternative à setTimeout."
---

# timer() - commence à publier après un délai

`timer()` est une Creation Function qui commence à publier des valeurs après un délai spécifié, à la fois une fois ou périodiquement.

## Vue d'ensemble.

`timer()` est une Creation Function flexible qui vous permet de contrôler le timing de la première émission. Son comportement dépend du nombre d'arguments et peut être soit une émission unique, soit une émission périodique comme `interval()`.

**Signature** :.

```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

**Documentation officielle** : [📘 RxJS formula : timer()](https://rxjs.dev/api/index/function/timer)

## Utilisation de base

Le comportement de `timer()` dépend du nombre d'arguments.

### Problème unique.

Si seul le premier argument est spécifié, 0 est émis après le temps spécifié pour terminer.

```typescript
import { timer } from 'rxjs';

// 3secondes après0Émettre et compléter le
const timer$ = timer(3000);

timer$.subscribe({
  next: value => console.log('Valeur:', value),
  complete: () => console.log('Terminé')
});

// 3Sortie après secondes:
// Valeur: 0
// Terminé
```

### Émission périodique.

Si un intervalle est spécifié dans le deuxième argument, il continuera à émettre périodiquement après le délai initial.

```typescript
import { timer } from 'rxjs';

// 3Commence après secondes et ensuite1Émission d'une valeur toutes les secondes
const timer$ = timer(3000, 1000);

timer$.subscribe(value => console.log('Valeur:', value));

// Sortie:
// Valeur: 0  (3(après secondes)
// Valeur: 1  (4(après secondes)
// Valeur: 2  (5(après secondes)
// ...(continue indéfiniment)
```

## Caractéristiques importantes.

### 1. Spécification flexible des délais

Le délai peut être spécifié comme un nombre en millisecondes ou comme un objet `Date`.

```typescript
import { timer } from 'rxjs';

// Spécifié en millisecondes
timer(5000).subscribe(() => console.log('5Après les secondes'));

// Date Spécifié par l'objet (exécuté à un moment précis)
const targetTime = new Date(Date.now() + 10000); // 10Après les secondes
timer(targetTime).subscribe(() => console.log('Exécuté à l'heure spécifiée'));
```

### Le comportement change avec ou sans le deuxième argument.

Le fait que le deuxième argument soit spécifié ou non détermine l'achèvement ou non de l'opération.

```typescript
import { timer } from 'rxjs';

// Spécifié par l'objet en millisecondes (continue indéfiniment)2Pas d'argument - 1Émis une fois et terminé
timer(1000).subscribe({
  next: value => console.log('12ème:', value),
  complete: () => console.log('Terminé')
});

// Spécifié par l'objet en millisecondes (continue indéfiniment)2Avec argument - Continue à émettre indéfiniment
timer(1000, 1000).subscribe({
  next: value => console.log('Répétition:', value),
  complete: () => console.log('Terminé (non illustré)')
});
```

> [!IMPORTANT]

> **Incomplet avec le deuxième argument**
>
> > Si un second argument est spécifié, comme `timer(1000, 1000)`, il continuera à émettre indéfiniment, comme avec `interval()`. Il doit toujours être désabonné.

### 5. Observable à froid.

`timer()` est un Cold Observable, qui crée un timer indépendant pour chaque abonnement.

```typescript
import { timer } from 'rxjs';

const timer$ = timer(1000);

console.log('Commencer');

// S'abonner1
timer$.subscribe(() => console.log('Observer 1'));

// 500msS'abonner après2Ajouter
setTimeout(() => {
  timer$.subscribe(() => console.log('Observer 2'));
}, 500);

// Sortie:
// Commencer
// Observer 1  (1(après secondes)
// Observer 2  (1.5Après les secondes - (minuterie indépendante)
```

> [!NOTE]

> **Caractéristiques Observables à froid**
> Chaque abonnement démarre une exécution indépendante
> - Chaque abonné reçoit son propre flux de données
> - Un timer indépendant est démarré pour chaque abonnement ; comme avec interval()`, utilisez `share()` si le partage est nécessaire.
>
> > Pour plus d'informations, voir [Cold Observable et Hot Observable](/fr/guide/observables/cold-and-hot-observables).

## Différence entre timer() vs interval()

La principale différence entre les deux est le moment de la première question.

```typescript
import { timer, interval } from 'rxjs';
import { take } from 'rxjs';

console.log('Commencer');

// interval() - Commence immédiatement (après1(première valeur en secondes)
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - Pas de délai (première valeur immédiatement)
timer(0, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// timer() - 2Démarre après un délai de quelques secondes
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer(Retardé (première valeur immédiatement)):', value);
});
```

```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

## Cas d'utilisation pratique

### 1. Exécution paresseuse

Exécuter le processus une seule fois après un certain temps.


```typescript
import { from, timer } from 'rxjs';
import { switchMap } from 'rxjs';

function delayedApiCall() {
  return timer(2000).pipe(
    switchMap(() => from(
      fetch('https://jsonplaceholder.typicode.com/posts/1')
        .then(res => res.json())
    ))
  );
}

delayedApiCall().subscribe(data => {
  console.log('2Acquisition des données après quelques secondes:', data);
});
```

### Interrogation avec délai

La première fois, l'interrogation n'est pas effectuée immédiatement, mais commence après un certain temps.

```typescript
import { from, timer } from 'rxjs';
import { switchMap, retry } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

// 5Commence l'interrogation après quelques secondes, puis10toutes les secondes
const polling$ = timer(5000, 10000).pipe(
  switchMap(() => from(
    fetch('https://jsonplaceholder.typicode.com/users/1')
      .then(res => res.json() as Promise<Status>)
  )),
  retry(3) // En cas d'erreur3Réessaie jusqu'à trois fois
);

const subscription = polling$.subscribe(data => {
  console.log('Mise à jour de l'état:', data);
});

// S'arrête si nécessaire
// subscription.unsubscribe();
```

### 3. processus de temporisation

Délai d'attente si le traitement n'est pas terminé dans un certain laps de temps.

```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

### 4. masquage automatique des notifications

Masque automatiquement les notifications au bout d'un certain temps après leur affichage.


## Utilisation dans le pipeline.

Le ` timer()` est utilisé comme point de départ pour un traitement différé et une exécution périodique.


```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

## Erreurs courantes.

### 1. oublier de se désinscrire avec un deuxième argument


### 2. ne pas comprendre la différence entre interval() et


## Considérations sur les performances.

Bien que `timer()` soit léger, son utilisation peut affecter les performances.

> [!TIP]

> **Conseils d'optimisation** :.
> - Ne jamais spécifier un second argument pour une exécution unique.
> - Désabonnez-vous toujours lorsque vous n'en avez plus besoin
> - Partagez avec `share()` si plusieurs Observateurs sont nécessaires
> - Utilisez les intervalles courts (moins de 100 ms) avec prudence.


```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

## Fonctions de Creation Function liées.


## Résumé

- Timer()` est une Creation Function qui commence à émettre après un certain délai.
- Sans deuxième argument : émission unique (complète)
- Avec le second argument : émission périodique (ne se termine jamais)
- Le délai est spécifié en millisecondes ou en objet `Date`.
- Convient à l'exécution différée, à l'interrogation avec délai, au traitement des délais d'attente.

## Prochaine étape.

- [interval() - publier en continu à des intervalles spécifiés](/fr/guide/creation-functions/basic/interval)
- [defer() - retarde la génération de l'abonnement](/fr/guide/creation-functions/conditional/defer)
- [retour à l'aperçu des fonctions de création de base](/fr/guide/creation-functions/basic/)
