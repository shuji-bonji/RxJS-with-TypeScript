---
description: O operador elementAt é um operador de filtragem RxJS que recupera apenas o valor em uma posição de índice especificada. Comporta-se de forma semelhante ao acesso de índice de array.
---

# elementAt - Obter Valor no Índice Especificado

O operador `elementAt` recupera **apenas o valor na posição de índice especificada** de um Observable e completa imediatamente o stream. Comporta-se de forma semelhante a `array[index]`.

## 🔰 Sintaxe Básica e Uso

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Saída: 30 (valor no índice 2)
```

**Fluxo de operação**:
1. 10 (índice 0) → Pular
2. 20 (índice 1) → Pular
3. 30 (índice 2) → Emitir e completar
4. 40, 50 não são avaliados

[🌐 Documentação Oficial RxJS - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Padrões de Uso Típicos

- **Paginação**: Obter primeiro item de uma página específica
- **Recuperação de dados ordenados**: Obter N-ésimo evento ou mensagem
- **Teste e depuração**: Verificar valor em posição específica
- **Acesso semelhante a array**: Tratar Observable como um array

## 🧠 Exemplo de Código Prático: Contagem Regressiva de Evento

Exemplo de execução de uma ação no N-ésimo clique.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// Criar UI
const output = document.createElement('div');
output.innerHTML = '<h3>Exibir mensagem no 5º clique</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Clique';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Por favor, clique mais 5 vezes';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Evento de clique
const clicks$ = fromEvent(button, 'click');

// Exibição de contagem
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `${remaining} cliques restantes`;
  } else {
    counter.textContent = '';
  }
});

// Detectar 5º clique (índice 4)
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Conseguiu!';
  result.style.color = 'green';
  button.disabled = true;
});
```

- Completa no 5º clique (índice 4).
- Começa de 0, mesmo que índice de array.

## 🆚 Comparação com Operadores Similares

### elementAt vs take vs first

| Operador | Valor Recuperado | Contagem de Saída | Caso de Uso |
|:---|:---|:---|:---|
| `elementAt(n)` | Apenas valor no índice n | 1 | Obter N-ésimo valor |
| `take(n)` | Primeiros n valores | n | Obter primeiros N valores |
| `first()` | Primeiro valor | 1 | Obter primeiro |
| `skip(n) + first()` | Primeiro após pular n | 1 | Mesmo que elementAt (não recomendado) |

## ⚠️ Observações

### 1. Quando o Índice Está Fora do Intervalo

Se o índice especificado não for alcançado antes do stream completar, ocorre um erro.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // Apenas 3 itens

numbers$.pipe(
  elementAt(5) // Requisitar índice 5
).subscribe({
  next: console.log,
  error: err => console.error('Erro:', err.message)
});
// Saída: Erro: no elements in sequence
```

### 2. Especificar Valor Padrão

Você pode especificar um valor padrão para evitar erros.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Especificar valor padrão
numbers$.pipe(
  elementAt(5, 999) // Retornar 999 se índice 5 não existir
).subscribe({
  next: console.log,
  error: err => console.error('Erro:', err.message)
});
// Saída: 999
```

### 3. Uso com Streams Assíncronos

Para streams assíncronos, ele espera até alcançar a posição do índice.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// Emitir valor a cada segundo
interval(1000).pipe(
  elementAt(3) // Índice 3 (4º valor)
).subscribe(console.log);
// Saída após 3 segundos: 3
```

### 4. Índice Negativo Não Disponível

Índices negativos não podem ser especificados.

Para obter do final do array, use `takeLast` ou `last`.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Obter último valor
numbers$.pipe(
  last()
).subscribe(console.log);
// Saída: 50

// ✅ Obter últimos N valores
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Saída: 40, 50
```

## 📚 Operadores Relacionados

- **[take](/pt/guide/operators/filtering/take)** - Obter primeiros N valores
- **[first](/pt/guide/operators/filtering/first)** - Obter primeiro valor
- **[last](/pt/guide/operators/filtering/last)** - Obter último valor
- **[skip](/pt/guide/operators/filtering/skip)** - Pular primeiros N valores
- **[takeLast](/pt/guide/operators/filtering/takeLast)** - Obter últimos N valores

## Resumo

O operador `elementAt` recupera apenas o valor na posição de índice especificada.

- ✅ Mesmo comportamento que acesso de índice de array
- ✅ Ideal para obter N-ésimo valor
- ✅ Pode evitar erros especificando valor padrão
- ⚠️ Erro se índice estiver fora do intervalo (sem valor padrão)
- ⚠️ Índice negativo não disponível
- ⚠️ Espera até alcançar posição para streams assíncronos
